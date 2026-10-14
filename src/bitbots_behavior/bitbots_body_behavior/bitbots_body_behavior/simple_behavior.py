import builtins
import math
import time
from typing import NamedTuple

from bitbots_utils.transforms import quat_from_yaw
from geometry_msgs.msg import PoseStamped
from bitbots_msgs.msg import HeadMode


HEAD_MODE_TRACK_BALL = "TRACK_BALL"
HEAD_MODE_SEARCH_FIELD_FEATURES = "SEARCH_FIELD_FEATURES"
HEAD_MODE_LOOK_FORWARD = "LOOK_FORWARD"
HEAD_MODE_DONT_MOVE = "DONT_MOVE"
HEAD_MODE_SEARCH_BALL_PENALTY = "SEARCH_BALL_PENALTY"
HEAD_MODE_SEARCH_FRONT = "SEARCH_FRONT"

_HEAD_MODE_LOOKUP = {
    HEAD_MODE_TRACK_BALL: HeadMode.TRACK_BALL,
    HEAD_MODE_SEARCH_FIELD_FEATURES: HeadMode.SEARCH_FIELD_FEATURES,
    HEAD_MODE_LOOK_FORWARD: HeadMode.LOOK_FORWARD,
    HEAD_MODE_DONT_MOVE: HeadMode.DONT_MOVE,
    HEAD_MODE_SEARCH_BALL_PENALTY: HeadMode.SEARCH_BALL_PENALTY,
    HEAD_MODE_SEARCH_FRONT: HeadMode.SEARCH_FRONT,
}


class CurrentPose(NamedTuple):
    x: float
    y: float
    yaw_rad: float


class BallPose(NamedTuple):
    x: float
    y: float


class SimpleBehaviorRuntime:
    def __init__(self, node, blackboard):
        self._node = node
        self._blackboard = blackboard

    def set_goal_pose(self, x: float, y: float, yaw_deg: float = 0.0) -> None:
        """Set a navigation goal pose in map coordinates."""
        goal = PoseStamped()
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.header.frame_id = self._blackboard.world_model.map_frame
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation = quat_from_yaw(math.radians(float(yaw_deg)))
        self._blackboard.pathfinding.publish(goal)

    def kick_ball(self, direction_deg: float, strength: float = 3.0) -> bool:
        """Kick the ball in a map-frame direction in degrees and wait for completion."""
        kick_started = self._blackboard.kick.start_rl_kick(float(direction_deg), float(strength))
        if not kick_started:
            return False

        while self._blackboard.kick.is_currently_kicking:
            time.sleep(0.01)
        self._blackboard.world_model.forget_ball()
        return True

    def set_head_mode(self, mode: str | int) -> None:
        """Configure head mode by named constant or raw mode value."""
        if isinstance(mode, str):
            try:
                head_mode = _HEAD_MODE_LOOKUP[mode]
            except KeyError as error:
                raise ValueError(f"Unknown head mode '{mode}'") from error
        else:
            head_mode = int(mode)

        self._blackboard.misc.set_head_duty(head_mode)

    def stop(self) -> None:
        """Stop walking and cancel the current pathfinding goal."""
        self._blackboard.pathfinding.stop_walk()

    def get_current_pose(self) -> CurrentPose | None:
        """Get the current robot pose as named fields in map frame."""
        try:
            current_pose = self._blackboard.world_model.get_current_position()
            if current_pose is None:
                return None
            return CurrentPose(float(current_pose[0]), float(current_pose[1]), float(current_pose[2]))
        except Exception as error:
            self._node.get_logger().warn(f"Could not get current pose: {error}")
            return None

    def get_ball_pose(self) -> BallPose | None:
        """Get the ball position as named fields in map frame."""
        ball_pose = self._blackboard.world_model.get_ball_position_xy()
        if ball_pose is None:
            return None
        return BallPose(float(ball_pose[0]), float(ball_pose[1]))

    def print(self, *args, sep=" ", end="\n"):
        self._node.get_logger().info(sep.join(map(str, args)))


_runtime: SimpleBehaviorRuntime | None = None


def configure_runtime(runtime: SimpleBehaviorRuntime) -> None:
    """Configure the runtime used by the global API functions."""
    global _runtime
    _runtime = runtime


def _require_runtime() -> SimpleBehaviorRuntime:
    if _runtime is None:
        raise RuntimeError("Simple behavior runtime is not configured.")
    return _runtime


def set_goal_pose(x: float, y: float, yaw_deg: float = 0.0) -> None:
    """Set a navigation goal pose in map coordinates."""
    _require_runtime().set_goal_pose(x, y, yaw_deg)


def kick_ball(direction_deg: float, strength: float = 3.0) -> bool:
    """Kick the ball in a map-frame direction in degrees and wait for completion."""
    return _require_runtime().kick_ball(direction_deg, strength)


def set_head_mode(mode: str | int) -> None:
    """Configure head mode by named constant or raw mode value."""
    _require_runtime().set_head_mode(mode)


def stop() -> None:
    """Stop walking and cancel the current pathfinding goal."""
    _require_runtime().stop()


def get_current_pose() -> CurrentPose | None:
    """Get the current robot pose as named fields in map frame."""
    return _require_runtime().get_current_pose()


def get_ball_pose() -> BallPose | None:
    """Get the ball position as named fields in map frame."""
    return _require_runtime().get_ball_pose()

def print(*args, **kwargs):
    if _runtime is not None:
        _runtime.print(*args, **kwargs)
    else:
        builtins.print(*args, **kwargs)


__all__ = [
    "CurrentPose",
    "BallPose",
    "set_goal_pose",
    "kick_ball",
    "set_head_mode",
    "stop",
    "get_current_pose",
    "get_ball_pose",
    "HEAD_MODE_TRACK_BALL",
    "HEAD_MODE_SEARCH_FIELD_FEATURES",
    "HEAD_MODE_LOOK_FORWARD",
    "HEAD_MODE_DONT_MOVE",
    "HEAD_MODE_SEARCH_BALL_PENALTY",
    "HEAD_MODE_SEARCH_FRONT",
    "print",
]
