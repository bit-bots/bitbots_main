"""Copied simulation ground truth, independent of ROS message objects."""

from dataclasses import dataclass, field

Position = tuple[float, float, float]


@dataclass(frozen=True)
class RobotMotion:
    linear_speed: float
    angular_speed: float
    body_joint_speed: float
    head_joint_speed: float
    upright: float
    relative_height: float


@dataclass(frozen=True)
class SimulationObservation:
    time_ns: int
    step_number: int
    ball_position: Position | None
    robot_positions: dict[int, Position]
    touching_ball: frozenset[int]
    teleported_robots: frozenset[int] = frozenset()
    ball_teleported: bool = False
    robot_motion: dict[int, RobotMotion] = field(default_factory=dict)
