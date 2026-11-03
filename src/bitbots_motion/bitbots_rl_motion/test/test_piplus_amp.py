#!/usr/bin/env python3
"""Checks the Pi Plus AMP walk and kick policies against their configs.

The nodes give the policies a flat observation. A wrong joint count, a wrong
term order or a wrong history order does not cause an error. It only causes a
policy that behaves badly on the robot. These tests pin the layout.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import onnxruntime as rt
import pytest
import yaml  # type: ignore[import-untyped]
from bitbots_rl_motion.handlers.gait_clock_handler import GaitClockHandler
from bitbots_rl_motion.history_buffer import HistoryBuffer
from bitbots_rl_motion.piplus_amp_utils import (
    ActionLimiter,
    kick_frame,
    kick_intensity_from_strength,
    soft_joint_limits,
    stand_flag,
    walk_observation,
)

PACKAGE_ROOT = Path(__file__).parent.parent
URDF_PATH = PACKAGE_ROOT.parent.parent / "bitbots_robot" / "piplus_description" / "urdf" / "pi_plus_22dof.urdf.xacro"

# Joint order of the training tasks (order of the joints in the training robot model)
TRAINING_JOINT_ORDER = [
    "r_shoulder_pitch_joint",
    "r_shoulder_roll_joint",
    "r_upper_arm_joint",
    "r_elbow_joint",
    "l_shoulder_pitch_joint",
    "l_shoulder_roll_joint",
    "l_upper_arm_joint",
    "l_elbow_joint",
    "r_hip_pitch_joint",
    "r_hip_roll_joint",
    "r_thigh_joint",
    "r_calf_joint",
    "r_ankle_pitch_joint",
    "r_ankle_roll_joint",
    "l_hip_pitch_joint",
    "l_hip_roll_joint",
    "l_thigh_joint",
    "l_calf_joint",
    "l_ankle_pitch_joint",
    "l_ankle_roll_joint",
]
NUMBER_OF_JOINTS = len(TRAINING_JOINT_ORDER)

# Actor terms of the walk policy (name, size), in order
WALK_TERMS = [
    ("base_ang_vel", 3),
    ("projected_gravity", 3),
    ("joint_pos", NUMBER_OF_JOINTS),
    ("joint_vel", NUMBER_OF_JOINTS),
    ("actions", NUMBER_OF_JOINTS + 1),
    ("command", 3),
    ("gait_phase", 2),
    ("stand", 1),
]
# Actor terms of one frame of the kick policy (name, size), in order
KICK_TERMS = [
    ("base_ang_vel", 3),
    ("projected_gravity", 3),
    ("joint_pos", NUMBER_OF_JOINTS),
    ("joint_vel", NUMBER_OF_JOINTS),
    ("actions", NUMBER_OF_JOINTS),
    ("ball_pos_b", 3),
    ("kick_dir_b", 2),
    ("kick_intensity", 1),
]

CONFIGS = {
    "piplus_amp_walk_node": "piplus_amp_walk_model.yaml",
    "piplus_amp_kick_node": "piplus_amp_kick_model.yaml",
}


def load_params(node_name: str) -> dict:
    with open(PACKAGE_ROOT / "configs" / CONFIGS[node_name]) as f:
        return yaml.safe_load(f)[node_name]["ros__parameters"]


def load_session(node_name: str) -> rt.InferenceSession:
    model_path = PACKAGE_ROOT / "models" / load_params(node_name)["model"]
    assert model_path.is_file(), f"The configured model {model_path} is missing"
    return rt.InferenceSession(model_path, providers=["CPUExecutionProvider"])


def urdf_limits() -> dict[str, tuple[float, float]]:
    limits = {}
    for joint in ET.fromstring(URDF_PATH.read_text()).iter("joint"):
        limit = joint.find("limit")
        name = joint.get("name")
        if limit is not None and name is not None and limit.get("lower") is not None:
            limits[name] = (float(limit.get("lower", 0.0)), float(limit.get("upper", 0.0)))
    return limits


def marker_terms(terms: list[tuple[str, int]]) -> dict[str, np.ndarray]:
    """Gives each term a different constant value, so that its position in the observation is visible."""
    return {name: np.full(size, float(index + 1), dtype=np.float32) for index, (name, size) in enumerate(terms)}


def expected_layout(terms: list[tuple[str, int]]) -> np.ndarray:
    return np.concatenate([np.full(size, float(index + 1)) for index, (_, size) in enumerate(terms)])


# --- configs and models


@pytest.mark.parametrize("node_name", list(CONFIGS))
def test_joint_order_is_the_training_order(node_name: str):
    assert load_params(node_name)["joints"]["ordered_relevant_joint_names"] == TRAINING_JOINT_ORDER


@pytest.mark.parametrize("node_name", list(CONFIGS))
def test_every_joint_is_configured(node_name: str):
    joints = load_params(node_name)["joints"]
    for key in ("walkready_state", "kp", "kd", "action_scales", "position_limits_lower", "position_limits_upper"):
        assert len(joints[key]) == NUMBER_OF_JOINTS, f"{key} does not have one value for each joint"


@pytest.mark.parametrize("node_name", list(CONFIGS))
def test_joints_exist_on_the_robot_and_limits_are_inside_the_robot_limits(node_name: str):
    robot_limits = urdf_limits()
    joints = load_params(node_name)["joints"]
    for name, lower, upper in zip(
        joints["ordered_relevant_joint_names"],
        joints["position_limits_lower"],
        joints["position_limits_upper"],
        strict=True,
    ):
        assert name in robot_limits, f"{name} is not a joint of the robot description"
        robot_lower, robot_upper = robot_limits[name]
        assert robot_lower - 1e-6 <= lower < upper <= robot_upper + 1e-6, f"{name} limits are outside the robot limits"


@pytest.mark.parametrize("node_name", list(CONFIGS))
def test_default_pose_is_inside_the_soft_limits(node_name: str):
    joints = load_params(node_name)["joints"]
    lower, upper = soft_joint_limits(
        joints["position_limits_lower"], joints["position_limits_upper"], joints["soft_limit_factor"]
    )
    default = np.array(joints["walkready_state"])
    assert np.all(lower <= default) and np.all(default <= upper)


@pytest.mark.parametrize("node_name", list(CONFIGS))
def test_control_rate_is_the_training_rate(node_name: str):
    params = load_params(node_name)
    assert params["phase"]["control_dt"] == pytest.approx(0.02)
    assert not params["phase"]["use_phase"]


def test_walk_model_shapes():
    session = load_session("piplus_amp_walk_node")
    assert session.get_inputs()[0].shape[-1] == sum(size for _, size in WALK_TERMS)
    # One action for each joint and one action for the gait clock frequency
    assert session.get_outputs()[0].shape[-1] == NUMBER_OF_JOINTS + 1


def test_kick_model_shapes():
    session = load_session("piplus_amp_kick_node")
    history_length = load_params("piplus_amp_kick_node")["obs"]["history_length"]
    assert session.get_inputs()[0].shape[-1] == history_length * sum(size for _, size in KICK_TERMS)
    assert session.get_outputs()[0].shape[-1] == NUMBER_OF_JOINTS


@pytest.mark.parametrize("node_name", list(CONFIGS))
def test_models_give_finite_actions_for_the_default_pose(node_name: str):
    session = load_session(node_name)
    observation = np.zeros((1, session.get_inputs()[0].shape[-1]), dtype=np.float32)
    action = session.run(None, {session.get_inputs()[0].name: observation})[0]
    assert np.all(np.isfinite(action))


# --- observation builders


def test_walk_observation_order():
    terms = marker_terms(WALK_TERMS)
    # A non-zero command gives a stand flag of 0, so use the marker only to check the command slot
    observation = walk_observation(
        terms["base_ang_vel"],
        terms["projected_gravity"],
        terms["joint_pos"],
        terms["joint_vel"],
        terms["actions"],
        terms["command"],
        terms["gait_phase"],
    )
    expected = expected_layout(WALK_TERMS)
    expected[-1] = 0.0
    assert observation.dtype == np.float32
    np.testing.assert_array_equal(observation, expected)


def test_stand_flag():
    assert stand_flag(np.zeros(3))[0] == 1.0
    assert stand_flag(np.array([0.0, 0.0, 0.1]))[0] == 0.0
    assert stand_flag(np.array([-0.2, 0.0, 0.0]))[0] == 0.0


def test_kick_observation_is_time_major_with_the_oldest_frame_first():
    terms = marker_terms(KICK_TERMS)
    history_length = load_params("piplus_amp_kick_node")["obs"]["history_length"]
    frame_size = sum(size for _, size in KICK_TERMS)
    history = HistoryBuffer(history_length)
    for step in range(history_length):
        frame = kick_frame(*(terms[name] + 100.0 * step for name, _ in KICK_TERMS))
        assert frame.shape == (frame_size,)
        history.append(frame)
    observation = history.buffer.flatten()

    assert observation.shape == (history_length * frame_size,)
    for step in range(history_length):
        # Each frame holds all terms of one step. The oldest step comes first.
        np.testing.assert_array_equal(
            observation[step * frame_size : (step + 1) * frame_size], expected_layout(KICK_TERMS) + 100.0 * step
        )


# --- gait clock


def test_gait_clock_frequency_range():
    clock = GaitClockHandler(1.0, 2.0, 0.02)
    assert clock.frequency_from_action(0.0) == pytest.approx(1.5)
    assert clock.frequency_from_action(100.0) == pytest.approx(2.0)
    assert clock.frequency_from_action(-100.0) == pytest.approx(1.0)


def test_gait_clock_advances_and_wraps():
    clock = GaitClockHandler(1.0, 2.0, 0.02, initial_phase=0.0)
    np.testing.assert_allclose(clock.get_obs(), [0.0, 1.0], atol=1e-6)
    # 1.5 Hz for 0.02 s each step
    for _ in range(10):
        clock.advance(0.0)
    assert clock.get_phase() == pytest.approx(0.3)
    angle = 2.0 * np.pi * 0.3
    np.testing.assert_allclose(clock.get_obs(), [np.sin(angle), np.cos(angle)], atol=1e-6)
    for _ in range(30):
        clock.advance(0.0)
    assert 0.0 <= clock.get_phase() < 1.0
    assert clock.get_phase() == pytest.approx(0.2)
    clock.reset()
    assert clock.get_phase() == 0.0


# --- kick command and safety limits


def test_kick_intensity_is_the_inverse_of_the_training_map():
    # The training gives intensity 0 -> 2 m/s and intensity 0.5 -> 4 m/s
    assert kick_intensity_from_strength(1.0, 1.0) == 0.0
    assert kick_intensity_from_strength(2.0, 1.0) == 0.0
    assert kick_intensity_from_strength(4.0, 1.0) == pytest.approx(0.5)
    assert kick_intensity_from_strength(1e9, 0.8) == pytest.approx(0.8)


@pytest.mark.parametrize("node_name", list(CONFIGS))
def test_clipped_joint_targets_are_inside_the_soft_limits(node_name: str):
    joints = load_params(node_name)["joints"]
    limiter = ActionLimiter(
        joints["position_limits_lower"],
        joints["position_limits_upper"],
        joints["soft_limit_factor"],
        joints["walkready_state"],
        joints["action_scales"],
        joints["action_limit"],
    )
    soft_lower, soft_upper = soft_joint_limits(
        joints["position_limits_lower"], joints["position_limits_upper"], joints["soft_limit_factor"]
    )
    rng = np.random.default_rng(0)
    for _ in range(100):
        action = rng.normal(0.0, 1000.0, NUMBER_OF_JOINTS).astype(np.float32)
        clipped, _ = limiter.clip_joint_action(action)
        target = np.array(joints["walkready_state"]) + clipped * np.array(joints["action_scales"])
        assert np.all(target >= soft_lower - 1e-4) and np.all(target <= soft_upper + 1e-4)
    feedback, changed = limiter.bound_feedback(np.full(NUMBER_OF_JOINTS, 1e9, dtype=np.float32))
    assert changed and np.all(feedback == joints["action_limit"])
