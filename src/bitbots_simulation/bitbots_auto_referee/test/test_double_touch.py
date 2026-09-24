"""Significant restart contacts and current team eligibility."""

from dataclasses import replace
from unittest.mock import Mock

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.double_touch import DoubleTouchRules
from bitbots_auto_referee.rules.outside import FieldGeometry


def setup():
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    state = replace(MatchState.initial(config), state="STATE_PLAYING", stopped=False)
    rule = DoubleTouchRules({0: 1, 1: 1, 2: 1, 3: 2}, {0: 1, 1: 2, 2: 3, 3: 1}, FieldGeometry(), Mock())
    rule.arm(1)
    rule.check(state, sample(0))
    return state, rule


def sample(time, touches=(), force=10.0, positions=None):
    return SimulationObservation(
        time_ns=int(time * 1e9), step_number=int(time * 1000), ball_position=(0, 0, 0.1),
        robot_positions=positions if positions is not None else {i: (i * 0.5, 1, 0.4) for i in range(4)},
        touching_ball=frozenset(touches), ball_contact_forces={robot: force for robot in touches},
    )


def test_separate_significant_touches_foul():
    state, rule = setup()
    assert rule.check(state, sample(0.1, (0,))) is None
    assert rule.check(state, sample(0.2)) is None
    assert rule.check(state, sample(0.3, (0,))) == 1
    assert rule.active_players == {1: 3, 2: 1}


def test_continuous_contact_and_short_gap_are_one_episode():
    state, rule = setup()
    for time in (0.1, 0.2, 0.3):
        assert rule.check(state, sample(time, (0,))) is None
    rule.check(state, sample(0.31))
    assert rule.check(state, sample(0.32, (0,))) is None


def test_small_force_and_small_impulse_do_not_count():
    state, rule = setup()
    rule.check(state, sample(0.1, (0,), force=0.1))
    assert rule.first_robot is None
    rule.check(state, sample(0.2))
    rule.check(state, sample(0.3))
    rule.check(state, sample(0.301, (0,)))
    assert rule.first_robot is None


def test_other_significant_robot_ends_restriction():
    for other in (1, 3):
        state, rule = setup()
        rule.check(state, sample(0.1, (0,)))
        rule.check(state, sample(0.2, (other,)))
        assert rule.check(state, sample(0.3, (0,))) is None
        assert rule.team_id is None


def test_current_penalty_and_presence_determine_eligibility():
    for excluded in ("penalized", "missing", "outside"):
        state, rule = setup()
        rule.check(state, sample(0.1, (0,)))
        rule.check(state, sample(0.2))
        observation = sample(0.3, (0,))
        if excluded == "penalized":
            home = state.teams[0]
            players = list(home.players)
            players[2] = replace(players[2], penalty="PENALTY_PUSHING")
            state = replace(state, teams=(replace(home, players=tuple(players)), state.teams[1]))
        elif excluded == "missing":
            del observation.robot_positions[2]
        else:
            observation.robot_positions[2] = (10, 0, 0.4)
        assert rule.check(state, observation) is None
        assert rule.active_players[1] == 2


def test_player_entering_field_before_second_touch_counts():
    state, rule = setup()
    first = sample(0.1, (0,))
    del first.robot_positions[2]
    rule.check(state, first)
    rule.check(state, sample(0.2))
    assert rule.check(state, sample(0.3, (0,))) == 1


def test_kickoff_arms_but_neutral_restart_does_not():
    for team in (1, 255):
        state, rule = setup()
        rule.check(replace(state, state="STATE_SET", kicking_team=team), sample(0.1))
        state = replace(state, kicking_team=team)
        rule.check(state, sample(0.2, (0,)))
        rule.check(state, sample(0.3))
        assert rule.check(state, sample(0.4, (0,))) == (1 if team == 1 else None)


def test_teleport_contact_is_suppressed_until_release():
    state, rule = setup()
    rule.check(state, replace(sample(0.1, (0,)), teleported_robots=frozenset({0})))
    rule.check(state, sample(0.2, (0,)))
    assert rule.first_robot is None
    rule.check(state, sample(0.3))
    rule.check(state, sample(0.4, (0,)))
    assert rule.first_robot == 0
