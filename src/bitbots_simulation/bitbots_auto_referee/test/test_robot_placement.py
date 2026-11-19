"""Standstill gating, STOP ownership and asynchronous robot-placement acknowledgements."""

from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock

import pytest

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import RobotMotion, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.check_rules import RuleChecker
from bitbots_auto_referee.rules.robot_placement import RobotPlacements

STILL = RobotMotion(0, 0, 0, 0, 1, 1)
MOVING = replace(STILL, body_joint_speed=1)


def sample(time, motion=STILL, **changes):
    return replace(
        SimulationObservation(
            int(time * 1e9), int(time * 1000), (0, 0, 0.1), {0: (0, 1, 0.4)}, frozenset(), robot_motion={0: motion}
        ),
        **changes,
    )


def setup(stopped=False, restart=False):
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    state = replace(MatchState.initial(config), state="STATE_PLAYING", stopped=stopped)
    if restart:
        state = replace(state, set_play="SET_PLAY_THROW_IN", secondary_time=45)
    commands = Mock()
    completed = Future()
    completed.set_result(TeleportResult(True, "placed"))
    commands.teleport_robot.return_value = completed
    checker = RuleChecker({0: config.home_team_id}, teleport_commands=commands)
    if restart:
        checker._restart_at = 0
    observation = sample(0, MOVING)
    state = checker.check_rules(state, observation)
    state = checker.motion_rules.penalize(state, observation, 0, "PENALTY_LEAVING_THE_FIELD")
    state = checker.check_rules(state, observation)
    return state, checker, commands


@pytest.mark.parametrize("channel", ["linear_speed", "angular_speed", "body_joint_speed", "head_joint_speed"])
@pytest.mark.parametrize("speed", [1.0, -1.0, float("nan"), float("inf")])
def test_each_motion_channel_blocks_placement(channel, speed):
    commands = Mock()
    placements = RobotPlacements(commands, Mock())
    placements.request(0, 1, 3.3, 0)
    placements.advance(sample(0, replace(STILL, **{channel: speed})))
    commands.teleport_robot.assert_not_called()
    assert placements.pending


def test_missing_motion_or_robot_does_not_grant_permission_to_teleport():
    commands = Mock()
    placements = RobotPlacements(commands, Mock())
    placements.request(0, 1, 3.3, 0)
    for observation in (sample(0, robot_motion={}), sample(1, robot_positions={})):
        placements.advance(observation)
    commands.teleport_robot.assert_not_called()


def test_stop_waits_for_motion_then_ack_and_observed_application_before_resuming():
    state, checker, commands = setup(restart=True)
    assert state.stopped
    remaining = state.secs_remaining
    pending = Future()
    commands.teleport_robot.return_value = pending
    state = checker.check_rules(state, sample(10, MOVING))
    assert state.stopped and state.secs_remaining == remaining and state.secondary_time == 45
    assert state.teams[0].players[0].secs_till_unpenalized == 45
    commands.teleport_robot.assert_not_called()
    state = checker.check_rules(state, sample(11))
    commands.teleport_robot.assert_called_once()
    assert state.stopped
    pending.set_result(TeleportResult(True, "placed", 12000))
    state = checker.check_rules(state, sample(11.5))
    assert state.stopped
    state = checker.check_rules(state, sample(12))
    assert not state.stopped and state.secondary_time == 45 and state.secs_remaining == remaining
    state = checker.check_rules(state, sample(13))
    assert state.secondary_time == 44 and state.secs_remaining == remaining - 1
    assert state.teams[0].players[0].secs_till_unpenalized == 44
    commands.teleport_robot.assert_called_once()


def test_robot_placement_preserves_preexisting_stop():
    state, checker, commands = setup(stopped=True)
    state = checker.check_rules(state, sample(1))
    assert state.stopped
    commands.teleport_robot.assert_called_once()


@pytest.mark.parametrize("failure", ["result", "exception"])
def test_failed_placement_keeps_stop_without_retrying(failure):
    state, checker, commands = setup()
    pending = Future()
    if failure == "result":
        pending.set_result(TeleportResult(False, "unavailable"))
    else:
        pending.set_exception(RuntimeError("unavailable"))
    commands.teleport_robot.return_value = pending
    for time in (1, 2, 3):
        state = checker.check_rules(state, sample(time))
        assert state.stopped
    commands.teleport_robot.assert_called_once()


def test_simulation_reset_cancels_stale_placement():
    state, checker, commands = setup()
    state = checker.check_rules(state, sample(10, MOVING))
    state = checker.check_rules(state, sample(0))
    assert not checker.robot_placements.pending
    assert not state.stopped
    commands.teleport_robot.assert_not_called()


def test_initial_cancels_placement_without_teleporting():
    state, checker, commands = setup()
    state = checker.check_rules(replace(state, state="STATE_INITIAL", stopped=True), sample(1))
    assert not checker.robot_placements.pending
    assert state.state == "STATE_INITIAL" and state.stopped
    commands.teleport_robot.assert_not_called()


def test_multiple_robots_wait_independently_and_resume_only_when_all_are_placed():
    state, checker, commands = setup()
    checker.teleport_robot(1, 1, -3.3, 0)
    state = checker.check_rules(
        state, sample(1, robot_positions={0: (0, 1, 0.4), 1: (1, 0, 0.4)}, robot_motion={0: STILL, 1: MOVING})
    )
    assert state.stopped
    commands.teleport_robot.assert_called_once()
    assert commands.teleport_robot.call_args.args[0] == 0
    state = checker.check_rules(
        state, sample(2, robot_positions={0: (0, 1, 0.4), 1: (1, 0, 0.4)}, robot_motion={0: STILL, 1: STILL})
    )
    assert not state.stopped
    assert commands.teleport_robot.call_count == 2
