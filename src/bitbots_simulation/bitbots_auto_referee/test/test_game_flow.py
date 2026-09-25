"""Game-flow decisions and acknowledged neutral restarts using synthetic observations."""

import math
from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import RobotMotion, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.check_rules import RuleChecker

UP = RobotMotion(0, 0, 0, 0, 1, 1)
DOWN = replace(UP, upright=0.1, relative_height=0.3)


def setup():
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    state = replace(MatchState.initial(config), state="STATE_PLAYING", stopped=False)
    commands = Mock()
    completed = Future()
    completed.set_result(TeleportResult(True, "placed"))
    commands.teleport_robot.return_value = completed
    commands.teleport_ball.return_value = completed
    return state, RuleChecker({0: 1}, teleport_commands=commands), commands


def sample(time, motion=UP, position=(0.3, 0, 0.4), ball=(0, 0, 0.1), blockage=0.0, **changes):
    return SimulationObservation(
        int(time * 1e9),
        int(time * 1000),
        ball,
        {0: position},
        frozenset(),
        robot_motion={0: motion},
        robot_yaws={0: math.pi},
        ball_blockage={0: blockage},
        **changes,
    )


def test_local_stuck_and_ball_motion_reset():
    state, checker, commands = setup()
    for time in (0, 9):
        state = checker.check_rules(state, sample(time))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"
    state = checker.check_rules(state, sample(10))
    assert state.teams[0].players[0].penalty == "PENALTY_LOCAL_GAME_STUCK"
    assert state.teams[0].players[0].secs_till_unpenalized == 45
    commands.teleport_robot.assert_called_once()
    state, checker, _ = setup()
    for time, ball in ((0, (0, 0, 0.1)), (9, (0.1, 0, 0.1)), (10, (0.1, 0, 0.1))):
        state = checker.check_rules(state, sample(time, ball=ball))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"


def test_global_restart_waits_for_ack_and_preserves_match():
    state, checker, commands = setup()
    state = replace(state, teams=tuple(replace(team, score=2) for team in state.teams))
    pending = Future()
    commands.teleport_ball.return_value = pending
    for time in (0, 30):
        state = checker.check_rules(state, sample(time, position=(2, -3.25, 0.4)))
    assert state.stopped and state.kicking_team == 255
    assert state.secs_remaining == 570 and state.teams[0].score == 2
    commands.teleport_robot.assert_called_once_with(0, 2, -3.25, math.pi)
    state = checker.check_rules(state, sample(31, position=(2, -3.25, 0.4)))
    assert state.stopped
    pending.set_result(TeleportResult(True, "placed", 32000))
    state = checker.check_rules(state, sample(31.5, position=(2, -3.25, 0.4)))
    assert state.stopped
    state = checker.check_rules(state, sample(32, position=(2, -3.25, 0.4), ball_teleported=True))
    assert state.state == "STATE_READY" and state.kicking_team == 255
    state = checker.check_rules(state, sample(57, position=(2, -3.25, 0.4)))
    assert state.state == "STATE_SET"
    state = checker.check_rules(state, sample(62, position=(2, -3.25, 0.4)))
    assert state.state == "STATE_PLAYING" and not state.stopped and state.secs_remaining == 570
    assert state.teams[0].score == 2


def test_set_play_does_not_accumulate_stuck_time():
    state, checker, _ = setup()
    state = replace(state, set_play="SET_PLAY_THROW_IN")
    for time in (0, 40):
        state = checker.check_rules(state, sample(time))
    assert not state.stopped and state.teams[0].players[0].penalty == "PENALTY_NONE"
    state = checker.check_rules(replace(state, set_play="SET_PLAY_NONE"), sample(41))
    assert not state.stopped and state.teams[0].players[0].penalty == "PENALTY_NONE"


def test_incapable_duration_and_failed_attempts():
    state, checker, _ = setup()
    state = replace(state, state="STATE_READY")
    for time in (0, 19, 20):
        state = checker.check_rules(state, sample(time, DOWN))
    assert state.teams[0].players[0].penalty == "PENALTY_INCAPABLE_ROBOT"
    state, checker, _ = setup()
    state = replace(state, state="STATE_READY")
    rising = replace(UP, upright=0.75, relative_height=0.7)
    for time, motion in enumerate((DOWN, rising, DOWN, rising, DOWN, rising, DOWN)):
        state = checker.check_rules(state, sample(time, motion))
    assert state.teams[0].players[0].penalty == "PENALTY_INCAPABLE_ROBOT"


def test_successful_recovery_resets_failure_history():
    state, checker, _ = setup()
    state = replace(state, state="STATE_READY")
    for time, motion in ((0, DOWN), (1, UP), (2, UP), (19, DOWN), (20, DOWN)):
        state = checker.check_rules(state, sample(time, motion))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"


def test_holding_requires_sustained_upright_blockage():
    state, checker, commands = setup()
    for time in (0, 4, 5):
        state = checker.check_rules(state, sample(time, blockage=1.0))
    assert state.teams[0].players[0].penalty == "PENALTY_BALL_HOLDING"
    commands.teleport_robot.assert_called_once()
    for motion, coverage in ((DOWN, 1.0), (UP, 0.5)):
        state, checker, _ = setup()
        for time in (0, 5):
            state = checker.check_rules(state, sample(time, motion, blockage=coverage))
        assert state.teams[0].players[0].penalty == "PENALTY_NONE"


def test_teleports_and_clock_reset_clear_stuck_history():
    state, checker, _ = setup()
    state = checker.check_rules(state, sample(0))
    state = checker.check_rules(state, sample(9))
    state = checker.check_rules(state, sample(10, ball_teleported=True))
    state = checker.check_rules(state, sample(0))
    state = checker.check_rules(state, sample(1))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"


def test_failed_global_placement_remains_stopped_without_repeated_commands():
    state, checker, commands = setup()
    failed = Future()
    failed.set_result(TeleportResult(False, "unavailable"))
    commands.teleport_ball.return_value = failed
    for time in (0, 30, 31, 32):
        state = checker.check_rules(state, sample(time, position=(2, -3.25, 0.4)))
    assert state.stopped and state.kicking_team == 255
    assert commands.teleport_ball.call_count == 1


def test_clock_reset_retries_incomplete_global_restart():
    state, checker, commands = setup()
    commands.teleport_ball.return_value = Future()
    for time in (0, 30, 0):
        state = checker.check_rules(state, sample(time, position=(2, -3.25, 0.4)))
    assert state.stopped
    assert commands.teleport_ball.call_count == 2
