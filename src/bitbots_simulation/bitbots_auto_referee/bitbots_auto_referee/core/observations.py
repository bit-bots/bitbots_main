"""Copied simulation ground truth, independent of ROS message objects."""

from dataclasses import dataclass, field

Position = tuple[float, float, float]


@dataclass(frozen=True)
class RobotBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True)
class RobotContact:
    robot_a: int
    robot_b: int
    force: float
    approach_a: float
    approach_b: float
    position: Position | None = None


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
    robot_bounds: dict[int, RobotBounds] = field(default_factory=dict)
    robot_yaws: dict[int, float] = field(default_factory=dict)
    ball_blockage: dict[int, float] = field(default_factory=dict)
    robot_contacts: tuple[RobotContact, ...] = ()
    ball_contact_forces: dict[int, float] = field(default_factory=dict)
