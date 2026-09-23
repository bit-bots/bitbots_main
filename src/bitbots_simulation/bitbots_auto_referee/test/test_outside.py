"""Regression cases for exit geometry and simulation-driven restart decisions."""

from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.check_rules import RuleChecker
from bitbots_auto_referee.rules.outside import FieldGeometry


def state():
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    return replace(MatchState.initial(config), state="STATE_PLAYING", stopped=False)


def sample(seconds, ball, **kwargs):
    return SimulationObservation(int(seconds * 1e9), int(seconds * 1000), ball, {}, frozenset(), **kwargs)


def test_whole_ball_and_marking_must_clear_boundary():
    field = FieldGeometry()
    assert not field.outside((4.55, 0.0, 0.1))
    assert field.outside((4.6, 0.0, 0.1))


def test_goal_height_sides_and_own_goal():
    field = FieldGeometry()
    match = state()
    for last_touch in (1, 2, None):
        decision = field.classify((4.0, 0.0, 0.1), (5.0, 0.0, 0.1), match, last_touch, 0)
        assert (decision.kind, decision.team_id) == ("GOAL", 2)
    decision = field.classify((4.0, 0.0, 2.0), (5.0, 0.0, 2.0), match, 2, 0)
    assert decision.kind == "SET_PLAY_GOAL_KICK"
    decision = field.classify((4.0, 0.0, 0.1), (5.0, 0.0, 0.1), replace(match, first_half=False), 1, 0)
    assert decision.team_id == 1


def test_restart_awards_and_first_crossing():
    field = FieldGeometry()
    match = state()
    corner = field.classify((4.0, 2.0, 0.1), (5.0, 2.0, 0.1), match, 1, 0)
    assert (corner.kind, corner.team_id, corner.position) == ("SET_PLAY_CORNER_KICK", 2, (4.5, 3.0))
    kick = field.classify((4.0, -2.0, 0.1), (5.0, -2.0, 0.1), match, 2, 0)
    assert (kick.kind, kick.team_id, kick.position) == ("SET_PLAY_GOAL_KICK", 1, (3.5, -1.5))
    throw = field.classify((4.0, 3.0, 0.1), (6.0, 4.0, 0.1), match, 1, 0)
    assert (throw.kind, throw.team_id) == ("SET_PLAY_THROW_IN", 2)
    assert field.classify((0.0, 3.0, 0.1), (0.0, 4.0, 0.1), match, None, 0) is None


def test_goal_delay_acknowledgement_and_ready_set_playing():
    commands = Mock()
    result = Future()
    commands.teleport_ball.return_value = result
    checker = RuleChecker({}, teleport_commands=commands)
    match = checker.check_rules(state(), sample(0, (4.0, 0.0, 0.1)))
    match = checker.check_rules(match, sample(1, (5.0, 0.0, 0.1)))
    match = checker.check_rules(match, sample(2, (5.0, 0.0, 0.1)))
    assert match.teams[1].score == 0
    commands.teleport_ball.assert_not_called()
    match = checker.check_rules(match, sample(3, (5.0, 0.0, 0.1)))
    assert match.teams[1].score == 1 and match.stopped and match.kicking_team == 1
    commands.teleport_ball.assert_called_once_with(0.0, 0.0, 0.0)
    result.set_result(TeleportResult(True, "placed", applied_step=4000))
    match = checker.check_rules(match, sample(3.5, (5.0, 0.0, 0.1)))
    assert match.stopped
    match = checker.check_rules(match, sample(4, (0.0, 0.0, 0.1), ball_teleported=True))
    assert match.state == "STATE_READY"
    match = checker.check_rules(match, sample(29, (0.0, 0.0, 0.1)))
    assert match.state == "STATE_SET"
    match = checker.check_rules(match, sample(34, (0.0, 0.0, 0.1)))
    assert match.state == "STATE_PLAYING" and not match.stopped
    assert match.teams[1].score == 1


def test_failed_placement_does_not_repeat_score_or_command():
    commands = Mock()
    result = Future()
    result.set_result(TeleportResult(False, "unavailable"))
    commands.teleport_ball.return_value = result
    checker = RuleChecker({}, teleport_commands=commands)
    match = checker.check_rules(state(), sample(0, (4.0, 0.0, 0.1)))
    for seconds in (1, 3, 4, 10):
        match = checker.check_rules(match, sample(seconds, (5.0, 0.0, 0.1)))
    assert match.stopped and match.teams[1].score == 1
    assert commands.teleport_ball.call_count == 1


def test_external_teleport_does_not_score():
    checker = RuleChecker({})
    match = checker.check_rules(state(), sample(0, (0.0, 0.0, 0.1)))
    match = checker.check_rules(match, sample(1, (5.0, 0.0, 0.1), ball_teleported=True))
    match = checker.check_rules(match, sample(4, (5.0, 0.0, 0.1)))
    assert match.teams[1].score == 0


def test_set_play_counts_down_and_rearms_for_next_exit():
    commands = Mock()
    result = Future()
    commands.teleport_ball.return_value = result
    checker = RuleChecker({}, teleport_commands=commands)
    checker.last_touch_team_id = 1
    match = checker.check_rules(state(), sample(0, (0.0, 3.0, 0.1)))
    match = checker.check_rules(match, sample(1, (0.0, 4.0, 0.1)))
    match = checker.check_rules(match, sample(3, (0.0, 4.0, 0.1)))
    assert match.stopped and match.secondary_time == 45
    result.set_result(TeleportResult(True, "placed", applied_step=4000))
    match = checker.check_rules(match, sample(4, (0.0, 3.0, 0.1), ball_teleported=True))
    assert match.set_play == "SET_PLAY_THROW_IN" and match.kicking_team == 2 and not match.stopped
    remaining = match.secs_remaining
    match = checker.check_rules(match, sample(5, (0.0, 3.0, 0.1)))
    assert match.secondary_time == 44
    assert match.secs_remaining == remaining - 1
    match = checker.check_rules(match, sample(49, (0.0, 3.0, 0.1)))
    assert match.set_play == "SET_PLAY_NONE" and match.secondary_time == 0
    checker.last_touch_team_id = 2
    match = checker.check_rules(match, sample(50, (0.0, 4.0, 0.1)))
    match = checker.check_rules(match, sample(52, (0.0, 4.0, 0.1)))
    assert match.kicking_team == 1 and commands.teleport_ball.call_count == 2


def test_default_home_side_and_explicit_override():
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    assert not config.home_defends_negative_x
    field = FieldGeometry.from_config(config)
    decision = field.classify((-4.0, 0.0, 0.1), (-5.0, 0.0, 0.1), state(), None, 0)
    assert decision.team_id == config.home_team_id
    field = replace(field, home_defends_negative_x=True)
    decision = field.classify((-4.0, 0.0, 0.1), (-5.0, 0.0, 0.1), state(), None, 0)
    assert decision.team_id == config.away_team_id


def test_delayed_service_response_resumes_after_teleport_observation():
    commands = Mock()
    result = Future()
    commands.teleport_ball.return_value = result
    checker = RuleChecker({}, teleport_commands=commands)
    checker.last_touch_team_id = 1
    match = checker.check_rules(state(), sample(0, (0.0, 3.0, 0.1)))
    match = checker.check_rules(match, sample(1, (0.0, 4.0, 0.1)))
    match = checker.check_rules(match, sample(3, (0.0, 4.0, 0.1)))
    match = checker.check_rules(match, sample(4, (0.0, 3.0, 0.1), ball_teleported=True))
    assert match.stopped
    result.set_result(TeleportResult(True, "placed", applied_step=4000))
    match = checker.check_rules(match, sample(5, (0.0, 3.0, 0.1)))
    assert match.state == "STATE_PLAYING" and not match.stopped
    assert commands.teleport_ball.call_count == 1
