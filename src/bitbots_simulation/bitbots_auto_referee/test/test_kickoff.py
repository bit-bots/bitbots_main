"""Kickoff placement starts in SET and waits for confirmed simulation state."""

from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock

import pytest

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.check_rules import RuleChecker


def sample(time, **changes):
    return replace(
        SimulationObservation(int(time * 1e9), int(time * 1000), (2, 0, 0.1), {}, frozenset()), **changes
    )


def setup():
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    state = replace(MatchState.initial(config), state="STATE_READY", stopped=False)
    commands = Mock()
    pending = Future()
    commands.teleport_ball.return_value = pending
    return state, RuleChecker({}, teleport_commands=commands), commands, pending


def test_opening_places_ball_only_in_set_and_waits_for_applied_step():
    state, checker, commands, pending = setup()
    for time in (0, 20):
        state = checker.check_rules(state, sample(time))
        assert state.state == "STATE_READY"
        commands.teleport_ball.assert_not_called()
    state = checker.check_rules(replace(state, state="STATE_SET", secondary_time=5), sample(25))
    assert state.state == "STATE_SET" and state.stopped
    commands.teleport_ball.assert_called_once_with(0.0, 0.0, 0.0)
    pending.set_result(TeleportResult(True, "placed", 27000))
    state = checker.check_rules(state, sample(26))
    assert state.state == "STATE_SET" and state.stopped
    state = checker.check_rules(state, sample(27, ball_teleported=True))
    assert state.state == "STATE_SET" and not state.stopped
    assert not checker.kickoff_ball.waiting
    commands.teleport_ball.assert_called_once()


@pytest.mark.parametrize("failure", ["result", "exception"])
def test_failed_kickoff_placement_blocks_set_without_repeated_command(failure):
    state, checker, commands, pending = setup()
    state = checker.check_rules(state, sample(0))
    if failure == "result":
        pending.set_result(TeleportResult(False, "unavailable"))
    else:
        pending.set_exception(RuntimeError("unavailable"))
    state = replace(state, state="STATE_SET")
    for time in (25, 30, 100):
        state = checker.check_rules(state, sample(time))
        assert state.state == "STATE_SET" and state.stopped
        assert checker.kickoff_ball.waiting
    commands.teleport_ball.assert_called_once()
