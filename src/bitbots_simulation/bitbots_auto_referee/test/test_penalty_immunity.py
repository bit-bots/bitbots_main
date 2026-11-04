"""INITIAL and already penalized players cannot generate additional penalties."""

from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock

import pytest

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import RobotBounds, RobotContact, RobotMotion, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.check_rules import RuleChecker

STILL = RobotMotion(0, 0, 0, 0, 1, 1)


def setup(phase, stopped=False):
    values = {name: spec.default for name, spec in PARAMETERS.items()}
    values.update(home_team_id=1, away_team_id=2)
    state = replace(MatchState.initial(RefereeConfig.from_parameters(values)), state=phase, stopped=stopped)
    commands = Mock()
    done = Future()
    done.set_result(TeleportResult(True, "placed"))
    commands.teleport_robot.return_value = done
    commands.teleport_ball.return_value = done
    events = []
    return state, RuleChecker({0: 1, 1: 2}, teleport_commands=commands, event_callback=events.append), commands, events


def sample(time, position=(-8, 0, 0.4), motion=STILL, **changes):
    x, y, _ = position
    return replace(
        SimulationObservation(
            int(time * 1e9),
            int(time * 1000),
            (0, 0, 0.1),
            {0: position, 1: (2, 1, 0.4)},
            frozenset(),
            robot_motion={0: motion, 1: STILL},
            robot_yaws={0: 0, 1: 0},
            robot_bounds={0: RobotBounds(x - 0.1, x + 0.1, y - 0.1, y + 0.1)},
        ),
        **changes,
    )


def test_initial_never_penalizes_movement_position_recovery_or_pushing():
    state, checker, commands, events = setup("STATE_INITIAL", stopped=True)
    down_moving = replace(STILL, linear_speed=1.0, body_joint_speed=1.0, upright=0.1, relative_height=0.3)
    for time in (0, 1, 2, 30):
        state = checker.check_rules(
            state, sample(time, motion=down_moving, robot_contacts=(RobotContact(0, 1, 30, 0, 0.2),))
        )
    assert all(player.penalty == "PENALTY_NONE" for team in state.teams for player in team.players)
    assert not any("PENALTY_" in event or "Pushing:" in event for event in events)
    commands.teleport_robot.assert_not_called()
    commands.teleport_ball.assert_not_called()
    for time in (31, 50):
        state = checker.check_rules(
            replace(state, state="STATE_READY", stopped=False), sample(time, (-2, 1, 0.4), down_moving)
        )
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"
    state = checker.check_rules(state, sample(51, (-2, 1, 0.4), down_moving))
    assert state.teams[0].players[0].penalty == "PENALTY_INCAPABLE_ROBOT"


@pytest.mark.parametrize(
    "phase, stopped", [("STATE_SET", False), ("STATE_PLAYING", True), ("STATE_PLAYING", False)]
)
def test_penalized_sideline_player_gets_no_extra_penalty_or_teleport(phase, stopped):
    state, checker, commands, events = setup(phase, stopped)
    state = checker.motion_rules.penalize(state, sample(0), 0, "PENALTY_PUSHING")
    moving = replace(STILL, linear_speed=1.0, angular_speed=1.0, body_joint_speed=1.0, head_joint_speed=1.0)
    for time in (0, 1, 2, 10):
        state = checker.check_rules(state, sample(time, motion=moving))
    player = state.teams[0].players[0]
    assert player.penalty == "PENALTY_PUSHING" and player.secs_till_unpenalized == 35
    assert sum("PENALTY_" in event for event in events) == 1
    commands.teleport_robot.assert_called_once()


def test_penalized_victim_cannot_trigger_a_pushing_foul_against_another_robot():
    state, checker, commands, _ = setup("STATE_PLAYING")
    home, away = state.teams
    home = replace(home, players=(replace(home.players[0], penalty="PENALTY_PUSHING"), *home.players[1:]))
    state = replace(state, teams=(home, away))
    for time in (0, 0.1, 6):
        motion = STILL if time == 0 else replace(STILL, upright=0.4)
        state = checker.check_rules(
            state, sample(time, motion=motion, robot_contacts=(RobotContact(0, 1, 30, 0, 0.2),))
        )
    assert state.teams[1].players[0].penalty == "PENALTY_NONE"
    assert state.set_play == "SET_PLAY_NONE"
    commands.teleport_robot.assert_not_called()
    commands.teleport_ball.assert_not_called()


def test_direct_penalty_requests_are_ignored_during_initial():
    state, checker, commands, _ = setup("STATE_INITIAL")
    assert checker.motion_rules.penalize(state, sample(0), 0, "PENALTY_ILLEGAL_POSITIONING") is state
    commands.teleport_robot.assert_not_called()
