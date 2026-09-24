"""Whole-body placement, retreat exceptions and shared penalty expiry."""

from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import RobotBounds, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.check_rules import RuleChecker


def match(**changes):
    config = RefereeConfig.from_parameters({key: spec.default for key, spec in PARAMETERS.items()})
    return replace(MatchState.initial(config), **changes)


def sample(time, positions, ball=(0.0, 0.0, 0.1)):
    return SimulationObservation(
        int(time * 1e9), int(time * 1000), ball, positions, frozenset(),
        robot_bounds={robot: RobotBounds(x - 0.1, x + 0.1, y - 0.1, y + 0.1)
                      for robot, (x, y, z) in positions.items()},
    )


def checker(teams):
    commands = Mock()
    result = Future()
    result.set_result(TeleportResult(True, "placed"))
    commands.teleport_robot.return_value = result
    return RuleChecker(teams, teleport_commands=commands), commands


def test_set_allows_only_one_kicker_touching_circle():
    rules, commands = checker({0: 1, 1: 1, 2: 2})
    state = match(state="STATE_SET", stopped=False)
    positions = {0: (-0.2, 0, 0.4), 1: (0.3, 0, 0.4), 2: (-2, 0, 0.4)}
    for time in (0, 1):
        state = rules.check_rules(state, sample(time, positions))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"
    assert state.teams[0].players[1].penalty == "PENALTY_ILLEGAL_POSITIONING"
    assert state.teams[1].players[0].penalty == "PENALTY_NONE"
    assert commands.teleport_robot.call_count == 1


def test_body_crossing_center_line_outside_circle_is_illegal():
    rules, _ = checker({0: 1})
    state = match(state="STATE_SET", stopped=False)
    for time in (0, 1):
        state = rules.check_rules(state, sample(time, {0: (0.05, 2, 0.4)}))
    assert state.teams[0].players[0].penalty == "PENALTY_ILLEGAL_POSITIONING"


def test_defender_retreat_and_tangent_are_allowed_but_standing_is_not():
    for direction in ((0.1, 0), (0, 0.1), (0, 0), (-0.1, 0)):
        rules, _ = checker({0: 2})
        state = match(state="STATE_PLAYING", stopped=False, set_play="SET_PLAY_THROW_IN")
        for time in (0, 0.5, 1, 1.5):
            state = rules.check_rules(state, sample(time, {0: (1 + direction[0] * time, direction[1] * time, 0.4)}))
        expected = "PENALTY_NONE" if direction in ((0.1, 0), (0, 0.1)) else "PENALTY_ILLEGAL_POSITIONING"
        assert state.teams[1].players[0].penalty == expected


def test_goal_kick_defender_must_clear_entire_penalty_area():
    rules, _ = checker({0: 2})
    state = match(state="STATE_PLAYING", stopped=False, set_play="SET_PLAY_GOAL_KICK")
    for time in (0, 1):
        state = rules.check_rules(state, sample(time, {0: (2.45, 1.9, 0.4)}, ball=(4, -1.5, 0.1)))
    assert state.teams[1].players[0].penalty == "PENALTY_ILLEGAL_POSITIONING"


def test_leaving_uses_whole_body_and_expires_after_placement():
    rules, commands = checker({0: 1})
    state = match(state="STATE_PLAYING", stopped=False)
    state = rules.check_rules(state, sample(0, {0: (5.55, 0, 0.4)}))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"
    state = rules.check_rules(state, sample(1, {0: (5.7, 0, 0.4)}))
    assert state.teams[0].players[0].penalty == "PENALTY_LEAVING_THE_FIELD"
    assert state.teams[0].players[0].secs_till_unpenalized == 45
    commands.teleport_robot.assert_called_once()
    state = rules.check_rules(state, sample(46, {0: (2.5, 3, 0.4)}))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"


def test_reset_does_not_turn_position_jump_into_retreat():
    rules, _ = checker({0: 2})
    state = match(state="STATE_PLAYING", stopped=False, set_play="SET_PLAY_THROW_IN")
    state = rules.check_rules(state, sample(10, {0: (0.5, 0, 0.4)}))
    state = rules.check_rules(state, sample(0, {0: (1, 0, 0.4)}))
    state = rules.check_rules(state, sample(1, {0: (1, 0, 0.4)}))
    assert state.teams[1].players[0].penalty == "PENALTY_ILLEGAL_POSITIONING"
