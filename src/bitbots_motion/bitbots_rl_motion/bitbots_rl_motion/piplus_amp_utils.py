"""Shared helpers for the Pi Plus AMP walk and kick policies.

The functions in this module have no ROS dependencies, so the tests can check
the observation layout and the action limits without a running node.
See the "Pi Plus AMP walk and kick policies" page in bitbots_docs for the
complete observation tables.
"""

from collections.abc import Sequence
from typing import Optional

import numpy as np

# The training sets the stand flag to 1 and pauses the gait clock when the
# absolute value of each velocity command is less than this value
# (stand_flag_obs, GaitPhaseActionCfg.stand_threshold).
STAND_COMMAND_THRESHOLD = 1e-3

# Ball speed in m/s that the kick training gives to a kick intensity of 0.
# The training maps an intensity x in [0, 1) to the target ball speed
# KICK_MIN_BALL_SPEED / (1 - x) (kick_intensity_to_target_speed).
KICK_MIN_BALL_SPEED = 2.0


def is_standing(command: np.ndarray) -> bool:
    """Returns True if all velocity commands are zero."""
    return bool(np.max(np.abs(command[:3])) < STAND_COMMAND_THRESHOLD)


def stand_flag(command: np.ndarray) -> np.ndarray:
    """Returns [1.0] if all velocity commands are zero, else [0.0]."""
    return np.array([1.0 if is_standing(command) else 0.0], dtype=np.float32)


def walk_observation(
    gyro: np.ndarray,
    gravity: np.ndarray,
    joint_pos_rel: np.ndarray,
    joint_vel: np.ndarray,
    last_action: np.ndarray,
    command: np.ndarray,
    gait_phase_obs: np.ndarray,
) -> np.ndarray:
    """Builds the actor observation of the walk policy (one frame, no history).

    The order of the terms must be the same as the order of the actor terms in
    the training task. The stand flag comes from the command.
    """
    return np.hstack(
        [
            gyro,
            gravity,
            joint_pos_rel,
            joint_vel,
            last_action,
            command,
            gait_phase_obs,
            stand_flag(command),
        ]
    ).astype(np.float32)


def kick_frame(
    gyro: np.ndarray,
    gravity: np.ndarray,
    joint_pos_rel: np.ndarray,
    joint_vel: np.ndarray,
    last_action: np.ndarray,
    ball_pos: np.ndarray,
    kick_direction: np.ndarray,
    kick_intensity: np.ndarray,
) -> np.ndarray:
    """Builds one frame of the actor observation of the kick policy.

    The kick policy observes a history of these frames in time-major order.
    Use a HistoryBuffer and flatten its buffer (oldest frame first).
    """
    return np.hstack(
        [
            gyro,
            gravity,
            joint_pos_rel,
            joint_vel,
            last_action,
            ball_pos,
            kick_direction,
            kick_intensity,
        ]
    ).astype(np.float32)


def parse_optional_scale(value) -> Optional[float]:
    """Reads an optional positive scale from a ROS parameter value.

    A YAML null arrives as the string "null" in a ROS parameter. None, "null",
    "none" and an empty string give None (feature off). A number must be
    positive. Other values are configuration errors.
    """
    if value is None or (isinstance(value, str) and value.strip().lower() in ("null", "none", "")):
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Expected a positive number or null, got {value!r}")
    if value <= 0.0:
        raise ValueError(f"Expected a positive number or null, got {value!r}")
    return float(value)


def bounded_ball_position(position: np.ndarray, position_clip: float, tanh_scale: Optional[float]) -> np.ndarray:
    """Applies the bounds of the kick training to the ball position in the anchor frame.

    Each axis is clipped to +/- position_clip. If tanh_scale is not None, each
    axis then becomes tanh_scale * tanh(x / tanh_scale), as in the kick training
    with tanh_scale (object_pos_b_noisy_delayed_world). Only a policy that was
    trained with this transform may get it.
    """
    bounded = np.clip(position, -position_clip, position_clip)
    if tanh_scale is not None:
        bounded = tanh_scale * np.tanh(bounded / tanh_scale)
    return bounded.astype(np.float32)


def kick_intensity_from_strength(strength: float, max_intensity: float) -> float:
    """Converts the strength of a kick goal to the kick intensity of the policy.

    The strength is the target ball speed in m/s. This function uses the
    inverse of the mapping of the training. A strength at or below the minimum
    ball speed gives the intensity 0. The result is clipped to max_intensity.
    """
    if strength <= KICK_MIN_BALL_SPEED:
        return 0.0
    intensity = 1.0 - KICK_MIN_BALL_SPEED / strength
    return float(np.clip(intensity, 0.0, max_intensity))


def soft_joint_limits(
    lower: Sequence[float], upper: Sequence[float], soft_limit_factor: float
) -> tuple[np.ndarray, np.ndarray]:
    """Shrinks the joint ranges around their centers, as mjlab does for its soft limits."""
    lower_array = np.asarray(lower, dtype=np.float32)
    upper_array = np.asarray(upper, dtype=np.float32)
    center = (lower_array + upper_array) / 2.0
    half_range = (upper_array - lower_array) / 2.0 * soft_limit_factor
    return center - half_range, center + half_range


def action_bounds(
    limits_lower: np.ndarray,
    limits_upper: np.ndarray,
    default_pose: Sequence[float],
    action_scales: Sequence[float],
) -> tuple[np.ndarray, np.ndarray]:
    """Returns the action range that keeps the joint target inside the limits.

    The joint target is default_pose + action * action_scale. The kick training
    clips the action to this range (LimitedJointPositionAction).
    """
    default_array = np.asarray(default_pose, dtype=np.float32)
    scale_array = np.asarray(action_scales, dtype=np.float32)
    assert np.all(scale_array > 0.0), "Action scales must be positive"
    return (limits_lower - default_array) / scale_array, (limits_upper - default_array) / scale_array


class ActionLimiter:
    """Safety limits for the joint actions of the Pi Plus AMP policies.

    The actor output has no bounds. This class applies two limits:

      * The joint action is clipped so that the joint target stays inside the
        soft joint limits. This is the same clip as in the kick training.
      * The action that goes back into the observation ("last action") is
        clipped to +/- feedback_limit. The training feeds back the action
        without a clip. Thus this limit must be much larger than the actions
        of the trained policy. It only stops a feedback loop that diverges.
    """

    def __init__(
        self,
        limits_lower: Sequence[float],
        limits_upper: Sequence[float],
        soft_limit_factor: float,
        default_pose: Sequence[float],
        action_scales: Sequence[float],
        feedback_limit: float,
    ):
        soft_lower, soft_upper = soft_joint_limits(limits_lower, limits_upper, soft_limit_factor)
        self.lower, self.upper = action_bounds(soft_lower, soft_upper, default_pose, action_scales)
        assert np.all(self.lower <= 0.0) and np.all(self.upper >= 0.0), (
            "The default pose must be inside the soft joint limits"
        )
        self.feedback_limit = feedback_limit

    def clip_joint_action(self, joint_action: np.ndarray) -> tuple[np.ndarray, bool]:
        """Returns the clipped joint action and True if the clip changed a value."""
        clipped = np.clip(joint_action, self.lower, self.upper).astype(np.float32)
        return clipped, bool(np.any(clipped != joint_action))

    def bound_feedback(self, action: np.ndarray) -> tuple[np.ndarray, bool]:
        """Returns the bounded feedback action and True if the bound changed a value."""
        bounded = np.clip(action, -self.feedback_limit, self.feedback_limit).astype(np.float32)
        return bounded, bool(np.any(bounded != action))
