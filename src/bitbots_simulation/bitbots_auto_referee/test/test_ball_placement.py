"""Restart placement clearance, nearby alternatives and field boundaries."""

import math
from dataclasses import replace

from bitbots_auto_referee.core.observations import RobotBounds, SimulationObservation
from bitbots_auto_referee.rules.ball_placement import PLACEMENT_MARGIN, SEARCH_RADIUS, free_ball_position
from bitbots_auto_referee.rules.outside import FieldGeometry
from bitbots_auto_referee.rules.robot_position import distance


def observation(positions=None, bounds=None):
    return SimulationObservation(0, 0, None, positions or {}, frozenset(), robot_bounds=bounds or {})


def test_unblocked_position_is_preserved():
    assert free_ball_position((0.2, 0.3), observation(), FieldGeometry()) == (0.2, 0.3)


def test_search_clears_all_body_bounds_and_stays_near_target():
    field = FieldGeometry()
    boxes = {0: RobotBounds(-0.3, 0.3, -0.3, 0.3), 1: RobotBounds(0.3, 0.8, -0.5, 0.5)}
    data = observation({0: (0, 0, 0.4), 1: (0.6, 0, 0.4)}, boxes)
    position = free_ball_position((0, 0), data, field)
    assert position is not None and math.hypot(*position) <= SEARCH_RADIUS
    assert all(distance(box, *position) >= field.ball_radius + PLACEMENT_MARGIN for box in boxes.values())


def test_corner_and_touchline_alternatives_stay_inside_field():
    field = FieldGeometry()
    for target in ((0, 3), (4.5, 3), (-4.5, -3), (3.5, 1.5)):
        data = observation({0: (*target, 0.4)})
        position = free_ball_position(target, data, field)
        assert position is not None and position != target
        assert abs(position[0]) <= field.field_length / 2 and abs(position[1]) <= field.field_width / 2
        assert math.dist(position, target) <= SEARCH_RADIUS


def test_invalid_bounds_use_conservative_root_clearance():
    data = observation({0: (0, 0, 0.4)}, {0: RobotBounds(float("nan"), 0, 0, 0)})
    position = free_ball_position((0, 0), data, FieldGeometry())
    assert position is not None and math.hypot(*position) > 0.6
    assert free_ball_position((0, 0), replace(data, robot_positions={0: (float("nan"), 0, 0)}), FieldGeometry()) is None


def test_no_nearby_space_does_not_return_an_occupied_position():
    data = observation({0: (0, 0, 0.4)}, {0: RobotBounds(-2, 2, -2, 2)})
    assert free_ball_position((0, 0), data, FieldGeometry()) is None
