"""Copied simulation ground truth, independent of ROS message objects."""

from dataclasses import dataclass

Position = tuple[float, float, float]


@dataclass(frozen=True)
class SimulationObservation:
    time_ns: int
    step_number: int
    ball_position: Position | None
    robot_positions: dict[int, Position]
    touching_ball: frozenset[int]
    teleported_robots: frozenset[int] = frozenset()
    ball_teleported: bool = False
