import builtins
import math
from typing import NamedTuple

from bitbots_utils.transforms import quat_from_yaw
from game_controller_hsl_interfaces.msg import GameState
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

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

GAME_STATE_INITIAL = "INITIAL"
GAME_STATE_READY = "READY"
GAME_STATE_SET = "SET"
GAME_STATE_PLAYING = "PLAYING"
GAME_STATE_FINISHED = "FINISHED"

_GAME_STATE_LOOKUP = {
    GameState.STATE_INITIAL: GAME_STATE_INITIAL,
    GameState.STATE_READY: GAME_STATE_READY,
    GameState.STATE_SET: GAME_STATE_SET,
    GameState.STATE_PLAYING: GAME_STATE_PLAYING,
    GameState.STATE_FINISHED: GAME_STATE_FINISHED,
}


class CurrentPose(NamedTuple):
    x: float
    y: float
    yew_deg: float


class BallPose(NamedTuple):
    x: float
    y: float


Pose = CurrentPose | BallPose


class SimpleBehaviorRuntime:
    def __init__(self, node, blackboard):
        self._node = node
        self._blackboard = blackboard

    def sleep(self, seconds: float | int) -> None:
        """Sleep for a specified duration in seconds based on simulation time."""
        if seconds <= 0:
            return
        duration = Duration(seconds=float(seconds))
        self._node.get_clock().sleep_for(duration)

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
        start = self._node.get_clock().now()
        while (self._node.get_clock().now() - start).nanoseconds < 0.2e9:
            self.stop()
            self.sleep(0.005)
        kick_started = self._blackboard.kick.start_rl_kick(float(direction_deg), float(strength))
        if not kick_started:
            return False

        while self._blackboard.kick.is_currently_kicking:
            self.sleep(0.01)
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

    def get_current_pose(self) -> CurrentPose:
        """Get the current robot pose as named fields in map frame."""
        current_pose = self._blackboard.world_model.get_current_position()
        if current_pose is None:
            raise RuntimeError("Current robot pose is not available.")
        return CurrentPose(float(current_pose[0]), float(current_pose[1]), math.degrees(float(current_pose[2])))

    def get_ball_pose(self) -> BallPose | None:
        """Get the confident ball position as named fields in map frame."""
        if not self._blackboard.world_model.ball_has_been_seen():
            return None
        ball_pose = self._blackboard.world_model.get_ball_position_xy()
        if ball_pose is None:
            return None
        return BallPose(float(ball_pose[0]), float(ball_pose[1]))

    def get_game_state(self) -> str:
        """Get the current main game state as a string (e.g. 'INITIAL', 'READY', 'SET', 'PLAYING', 'FINISHED')."""
        return _GAME_STATE_LOOKUP.get(self._blackboard.gamestate.get_main_state(), "UNKNOWN")

    def is_game_state(self, state: str) -> bool:
        """Check if the current game state matches the given state name."""
        return self.get_game_state() == state

    def is_game_stopped(self) -> bool:
        """Check if the game is currently stopped by the game controller."""
        return self._blackboard.gamestate.is_stopped()

    def is_stopped(self) -> bool:
        """Check if the game is currently stopped by the game controller."""
        return self.is_game_stopped()

    def has_kick(self) -> bool:
        """Check if our team currently has the kickoff or free kick."""
        return self._blackboard.gamestate.has_kick()

    def get_seconds_remaining(self) -> float:
        """Get the remaining seconds of the current game half."""
        return self._blackboard.gamestate.get_seconds_remaining()

    def get_secondary_seconds_remaining(self) -> float:
        """Get the remaining secondary seconds (e.g. for kickoff or set play)."""
        return self._blackboard.gamestate.get_secondary_seconds_remaining()

    def get_own_score(self) -> int:
        """Get our team's current score."""
        return self._blackboard.gamestate.get_own_score()

    def get_rival_score(self) -> int:
        """Get the opposing team's current score."""
        return self._blackboard.gamestate.get_rival_score()

    def is_penalized(self) -> bool:
        """Check if this robot is currently penalized."""
        return self._blackboard.gamestate.get_is_penalized()

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


def sleep(seconds: float | int) -> None:
    """Sleep for a specified duration in seconds based on simulation time."""
    _require_runtime().sleep(seconds)


def set_goal_pose(x: float | int, y: float | int, yaw_deg: float | int = 0.0) -> None:
    """Set a navigation goal pose in map coordinates."""
    _require_runtime().set_goal_pose(float(x), float(y), float(yaw_deg))


def set_goal_pose_from_pose(pose: Pose, yaw_deg: float | int = 0.0) -> None:
    """Set a navigation goal from a pose-like value."""
    set_goal_pose(pose.x, pose.y, yaw_deg)


def kick_ball(direction_deg: float | int, strength: float | int = 3.0) -> bool:
    """Kick the ball in a map-frame direction in degrees and wait for completion."""
    return _require_runtime().kick_ball(float(direction_deg), float(strength))


def set_head_mode(mode: str | int) -> None:
    """Configure head mode by named constant or raw mode value."""
    _require_runtime().set_head_mode(mode)


def stop() -> None:
    """Stop walking and cancel the current pathfinding goal."""
    _require_runtime().stop()


def get_current_pose() -> CurrentPose:
    """Get the current robot pose as named fields in map frame."""
    return _require_runtime().get_current_pose()


def get_ball_pose() -> BallPose | None:
    """Get the confident ball position as named fields in map frame."""
    return _require_runtime().get_ball_pose()


def get_game_state() -> str:
    """Get the current main game state as a string (e.g. 'INITIAL', 'READY', 'SET', 'PLAYING', 'FINISHED')."""
    return _require_runtime().get_game_state()


def is_game_state(state: str) -> bool:
    """Check if the current game state matches the given state name."""
    return _require_runtime().is_game_state(state)


def is_game_stopped() -> bool:
    """Check if the game is currently stopped by the game controller."""
    return _require_runtime().is_game_stopped()


def is_stopped() -> bool:
    """Check if the game is currently stopped by the game controller."""
    return _require_runtime().is_stopped()


def has_kick() -> bool:
    """Check if our team currently has the kickoff or free kick."""
    return _require_runtime().has_kick()


def get_seconds_remaining() -> float:
    """Get the remaining seconds of the current game half."""
    return _require_runtime().get_seconds_remaining()


def get_secondary_seconds_remaining() -> float:
    """Get the remaining secondary seconds (e.g. for kickoff or set play)."""
    return _require_runtime().get_secondary_seconds_remaining()


def get_own_score() -> int:
    """Get our team's current score."""
    return _require_runtime().get_own_score()


def get_rival_score() -> int:
    """Get the opposing team's current score."""
    return _require_runtime().get_rival_score()


def is_penalized() -> bool:
    """Check if this robot is currently penalized."""
    return _require_runtime().is_penalized()


def make_pose(x: float | int, y: float | int) -> BallPose:
    """Create a pose from map-frame coordinates."""
    return BallPose(float(x), float(y))


def pose_distance(first: Pose, second: Pose) -> float:
    """Calculate the distance in meters between two poses."""
    return math.hypot(second.x - first.x, second.y - first.y)


def pose_add(first: Pose, second: Pose) -> BallPose:
    """Add the coordinates of two poses."""
    return BallPose(first.x + second.x, first.y + second.y)


def pose_subtract(first: Pose, second: Pose) -> BallPose:
    """Subtract the second pose coordinates from the first."""
    return BallPose(first.x - second.x, first.y - second.y)


def pose_direction(first: Pose, second: Pose) -> float:
    """Calculate the map-frame direction from the first pose to the second in degrees."""
    return math.degrees(math.atan2(second.y - first.y, second.x - first.x))


def pose_from_direction(origin: Pose, distance: float | int, direction_deg: float | int) -> BallPose:
    """Create a pose at a distance and map-frame direction from an origin."""
    direction_rad = math.radians(float(direction_deg))
    return BallPose(
        origin.x + float(distance) * math.cos(direction_rad),
        origin.y + float(distance) * math.sin(direction_rad),
    )


def print(*args, **kwargs):
    if _runtime is not None:
        _runtime.print(*args, **kwargs)
    else:
        builtins.print(*args, **kwargs)


__all__ = [
    "CurrentPose",
    "BallPose",
    "Pose",
    "sleep",
    "set_goal_pose",
    "set_goal_pose_from_pose",
    "kick_ball",
    "set_head_mode",
    "stop",
    "get_current_pose",
    "get_ball_pose",
    "get_game_state",
    "is_game_state",
    "is_game_stopped",
    "is_stopped",
    "has_kick",
    "get_seconds_remaining",
    "get_secondary_seconds_remaining",
    "get_own_score",
    "get_rival_score",
    "is_penalized",
    "make_pose",
    "pose_distance",
    "pose_add",
    "pose_subtract",
    "pose_direction",
    "pose_from_direction",
    "HEAD_MODE_TRACK_BALL",
    "HEAD_MODE_SEARCH_FIELD_FEATURES",
    "HEAD_MODE_LOOK_FORWARD",
    "HEAD_MODE_DONT_MOVE",
    "HEAD_MODE_SEARCH_BALL_PENALTY",
    "HEAD_MODE_SEARCH_FRONT",
    "GAME_STATE_INITIAL",
    "GAME_STATE_READY",
    "GAME_STATE_SET",
    "GAME_STATE_PLAYING",
    "GAME_STATE_FINISHED",
    "print",
]
