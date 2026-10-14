import math

from bitbots_utils.transforms import quat_from_yaw
from geometry_msgs.msg import PoseStamped


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
        """Kick the ball in a map-frame direction in degrees."""
        return self._blackboard.kick.start_rl_kick(float(direction_deg), float(strength))

    def get_current_pose(self) -> tuple[float, float, float] | None:
        """Get the current robot pose as `(x, y, yaw_rad)` in map frame."""
        try:
            return self._blackboard.world_model.get_current_position()
        except Exception as error:
            self._node.get_logger().warn(f"Could not get current pose: {error}")
            return None

    def get_ball_pose(self) -> tuple[float, float] | None:
        """Get the ball position as `(x, y)` in map frame."""
        ball_pose = self._blackboard.world_model.get_ball_position_xy()
        if ball_pose is None:
            return None
        return float(ball_pose[0]), float(ball_pose[1])


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
    """Kick the ball in a map-frame direction in degrees."""
    return _require_runtime().kick_ball(direction_deg, strength)


def get_current_pose() -> tuple[float, float, float] | None:
    """Get the current robot pose as `(x, y, yaw_rad)` in map frame."""
    return _require_runtime().get_current_pose()


def get_ball_pose() -> tuple[float, float] | None:
    """Get the ball position as `(x, y)` in map frame."""
    return _require_runtime().get_ball_pose()


__all__ = ["set_goal_pose", "kick_ball", "get_current_pose", "get_ball_pose"]
