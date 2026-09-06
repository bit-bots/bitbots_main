#!/usr/bin/env python3
"""Checks that the AMP configs and the policies they load agree with each other.

The runners feed the policies a flat observation, so a wrong joint count, a wrong
history length or a term that is missing from the observation does not fail loudly,
it just produces a policy that behaves badly on the robot. These tests pin the sizes
against the ONNX models that are shipped with the package.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import onnxruntime as rt
import pytest
import yaml  # type: ignore[import-untyped]

PACKAGE_ROOT = Path(__file__).parent.parent

# Number of values a single frame of the observation holds besides the joints, and how
# many values per joint it holds. The joints contribute their position, their velocity
# and the previous action, see the obs() of the corresponding node.
VALUES_PER_JOINT = 3
# amp_walk_node: angular velocity (3) + projected gravity (3) + command (3)
WALK_EXTRA_VALUES = 3 + 3 + 3
# amp_kick_node: angular velocity (3) + projected gravity (3) + ball position (3)
# + kick direction (2) + strong kick (1). The ball velocity is observed by the critic
# during training only, so the policy does not get it.
KICK_EXTRA_VALUES = 3 + 3 + 3 + 2 + 1

CONFIGS = {
    "amp_walk_node": ("amp_walk_model.yaml", WALK_EXTRA_VALUES),
    "amp_kick_node": ("amp_kick_model.yaml", KICK_EXTRA_VALUES),
}


def load_params(config_name: str, node_name: str) -> dict:
    with open(PACKAGE_ROOT / "configs" / config_name) as f:
        return yaml.safe_load(f)[node_name]["ros__parameters"]


@pytest.mark.parametrize(("node_name", "config_name"), [(n, c) for n, (c, _) in CONFIGS.items()])
def test_every_joint_is_fully_configured(node_name: str, config_name: str):
    params = load_params(config_name, node_name)
    joints = params["joints"]
    number_of_joints = len(joints["ordered_relevant_joint_names"])

    assert number_of_joints > 0
    for name in ("walkready_state", "kp", "kd", "action_scales", "position_limits_lower", "position_limits_upper"):
        assert len(joints[name]) == number_of_joints, f"{name} does not cover every joint"


@pytest.mark.parametrize(("node_name", "config_name"), [(n, c) for n, (c, _) in CONFIGS.items()])
def test_the_policy_is_run_at_the_rate_it_was_trained_with(node_name: str, config_name: str):
    params = load_params(config_name, node_name)
    # The AMP tasks step at 50 Hz
    assert params["phase"]["control_dt"] == pytest.approx(0.02)
    assert not params["phase"]["use_phase"]


@pytest.mark.parametrize(("node_name", "config_name", "extra_values"), [(n, c, e) for n, (c, e) in CONFIGS.items()])
def test_the_observation_and_the_action_fit_the_policy(node_name: str, config_name: str, extra_values: int):
    params = load_params(config_name, node_name)
    number_of_joints = len(params["joints"]["ordered_relevant_joint_names"])
    history_length = params["obs"]["history_length"]

    model_path = PACKAGE_ROOT / "models" / params["model"]
    assert model_path.is_file(), f"The configured model {model_path} is missing"
    session = rt.InferenceSession(model_path, providers=["CPUExecutionProvider"])

    expected_observation = history_length * (extra_values + VALUES_PER_JOINT * number_of_joints)
    assert session.get_inputs()[0].shape[1] == expected_observation
    assert session.get_outputs()[0].shape[1] == number_of_joints


@pytest.mark.parametrize(("node_name", "config_name"), [(n, c) for n, (c, _) in CONFIGS.items()])
def test_the_default_pose_is_reachable(node_name: str, config_name: str):
    """The policy adds its action on top of the default pose, so a default pose outside
    the joint limits would be clamped away and the policy would never reach its own
    reference pose."""
    joints = load_params(config_name, node_name)["joints"]

    for name, default, lower, upper in zip(
        joints["ordered_relevant_joint_names"],
        joints["walkready_state"],
        joints["position_limits_lower"],
        joints["position_limits_upper"],
        strict=True,
    ):
        assert lower < upper, f"{name} has an empty range"
        assert lower <= default <= upper, f"the default pose of {name} is outside its limits"


@pytest.mark.parametrize(("node_name", "config_name"), [(n, c) for n, (c, _) in CONFIGS.items()])
def test_the_limits_match_the_robot(node_name: str, config_name: str):
    """The limits are the ones of the robot description, so the clamp keeps the joints
    inside what the hardware can actually reach."""
    urdf = ET.fromstring(
        (
            PACKAGE_ROOT.parent.parent / "bitbots_robot" / "piplus_description" / "urdf" / "pi_plus_22dof.urdf.xacro"
        ).read_text()
    )
    robot_limits: dict[str, tuple[float, float]] = {}
    for joint in urdf.iter("joint"):
        limit = joint.find("limit")
        name = joint.get("name")
        if limit is None or name is None:
            continue
        lower, upper = limit.get("lower"), limit.get("upper")
        if lower is not None and upper is not None:
            robot_limits[name] = (float(lower), float(upper))

    joints = load_params(config_name, node_name)["joints"]
    for name, lower, upper in zip(
        joints["ordered_relevant_joint_names"],
        joints["position_limits_lower"],
        joints["position_limits_upper"],
        strict=True,
    ):
        assert (lower, upper) == pytest.approx(robot_limits[name]), f"{name} does not match the robot description"
