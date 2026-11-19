"""Find nearby restart positions with clearance from observed robot bodies."""

import math

from bitbots_auto_referee.core.observations import RobotBounds, SimulationObservation
from bitbots_auto_referee.rules.outside import FieldGeometry
from bitbots_auto_referee.rules.robot_position import distance

PLACEMENT_MARGIN = 0.15
FALLBACK_ROBOT_RADIUS = 0.4
SEARCH_RADIUS = 1.0
SEARCH_STEP = 0.05
SEARCH_DIRECTIONS = 64


def free_ball_position(
    target: tuple[float, float], observation: SimulationObservation, field: FieldGeometry
) -> tuple[float, float] | None:
    """Search outward from the intended position, keeping the center inside the field."""
    half_x, half_y = field.field_length / 2, field.field_width / 2
    origin = (max(-half_x, min(half_x, target[0])), max(-half_y, min(half_y, target[1])))
    obstacles: list[RobotBounds | tuple[float, float]] = []
    for robot, position in observation.robot_positions.items():
        box = observation.robot_bounds.get(robot)
        if (
            box is not None
            and all(math.isfinite(value) for value in vars(box).values())
            and box.min_x <= box.max_x
            and box.min_y <= box.max_y
        ):
            obstacles.append(box)
        elif all(math.isfinite(value) for value in position[:2]):
            obstacles.append(position[:2])
        else:
            return None

    def clear(x, y):
        return abs(x) <= half_x and abs(y) <= half_y and all(
            distance(obstacle, x, y) >= field.ball_radius + PLACEMENT_MARGIN
            if isinstance(obstacle, RobotBounds)
            else math.dist(obstacle, (x, y)) >= FALLBACK_ROBOT_RADIUS + field.ball_radius + PLACEMENT_MARGIN
            for obstacle in obstacles
        )

    if clear(*origin):
        return origin
    for ring in range(1, round(SEARCH_RADIUS / SEARCH_STEP) + 1):
        radius = ring * SEARCH_STEP
        for direction in range(SEARCH_DIRECTIONS):
            angle = math.tau * direction / SEARCH_DIRECTIONS
            candidate = (origin[0] + radius * math.cos(angle), origin[1] + radius * math.sin(angle))
            if clear(*candidate):
                return candidate
    return None
