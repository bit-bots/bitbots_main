"""Motion penalties and set-play completion without starting simulator processes."""

from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import RobotMotion, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.check_rules import RuleChecker

STILL = RobotMotion(0.0, 0.0, 0.0, 0.0, 1.0, 1.0)


def initial(**changes):
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    return replace(MatchState.initial(config), **changes)


def sample(seconds, motion=STILL, contacts=(), **changes):
    observation = SimulationObservation(
        int(seconds * 1e9),
        int(seconds * 1000),
        (0.0, 0.0, 0.1),
        {0: (0.0, 1.0, 0.4)},
        frozenset(contacts),
        robot_motion={0: motion},
    )
    return replace(observation, **changes)


def test_set_grace_head_movement_and_getting_up_are_allowed():
    checker = RuleChecker({0: 1})
    state = initial(state="STATE_SET", stopped=False)
    for time, motion in (
        (0, replace(STILL, linear_speed=1.0)),
        (0.5, replace(STILL, linear_speed=1.0)),
        (1, replace(STILL, head_joint_speed=1.0)),
        (2, replace(STILL, body_joint_speed=1.0, upright=0.2)),
        (3, replace(STILL, body_joint_speed=1.0, relative_height=0.3)),
        (3.5, replace(STILL, body_joint_speed=1.0)),
        (4, STILL),
    ):
        state = checker.check_rules(state, sample(time, motion))
        assert state.teams[0].players[0].penalty == "PENALTY_NONE"


def test_set_penalty_expires_without_repeated_extension():
    checker = RuleChecker({0: 1})
    state = initial(state="STATE_SET", stopped=False)
    moving = replace(STILL, body_joint_speed=1.0)
    for time in (0, 1, 1.25):
        state = checker.check_rules(state, sample(time, moving))
    assert state.teams[0].players[0].penalty == "PENALTY_MOTION_IN_SET"
    assert state.teams[0].players[0].secs_till_unpenalized == 15
    state = checker.check_rules(state, sample(2.25, moving))
    assert state.teams[0].players[0].secs_till_unpenalized == 14
    state = checker.check_rules(state, sample(16.25, moving))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"


def test_stop_includes_head_and_teleports_only_once():
    commands = Mock()
    future = Future()
    future.set_result(TeleportResult(True, "placed"))
    commands.teleport_robot.return_value = future
    checker = RuleChecker({0: 1}, teleport_commands=commands, robot_players={0: 2})
    state = initial(stopped=True)
    moving = replace(STILL, head_joint_speed=1.0)
    for time in (0, 1, 1.25, 2):
        state = checker.check_rules(state, sample(time, moving))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"
    assert state.teams[0].players[1].penalty == "PENALTY_MOTION_IN_STOP"
    commands.teleport_robot.assert_called_once()
    robot, x, y, yaw = commands.teleport_robot.call_args.args
    assert robot == 0 and x > 0 and y > 0 and yaw < 0
    state = checker.check_rules(state, sample(46.25))
    assert state.teams[0].players[1].penalty == "PENALTY_NONE"


def test_teleport_and_clock_reset_do_not_count_as_motion():
    checker = RuleChecker({0: 1})
    state = initial(state="STATE_SET", stopped=False)
    moving = replace(STILL, linear_speed=1.0)
    for time in (0, 1):
        state = checker.check_rules(state, sample(time, moving))
    state = checker.check_rules(state, sample(1.25, moving, teleported_robots=frozenset({0})))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"
    for time in (2.25, 2.5):
        state = checker.check_rules(state, sample(time, moving))
    assert state.teams[0].players[0].penalty == "PENALTY_MOTION_IN_SET"
    state = checker.check_rules(state, sample(0, moving))
    assert state.teams[0].players[0].penalty == "PENALTY_NONE"


def test_kicking_team_contact_clears_restart_and_stays_cleared():
    checker = RuleChecker({0: 1, 1: 2})
    state = initial(state="STATE_PLAYING", stopped=False, set_play="SET_PLAY_THROW_IN", secondary_time=45)
    checker._restart_at = 0
    state = checker.check_rules(state, sample(1, contacts=(1,), robot_positions={0: (0, 0, 0), 1: (1, 0, 0)}))
    assert state.set_play == "SET_PLAY_THROW_IN"
    state = checker.check_rules(state, sample(2, contacts=(0,)))
    assert state.set_play == "SET_PLAY_NONE" and state.secondary_time == 0
    state = checker.check_rules(state, sample(3))
    assert state.secondary_time == 0


def test_placement_contact_and_ready_do_not_end_restart():
    checker = RuleChecker({0: 1})
    state = initial(state="STATE_PLAYING", stopped=False, set_play="SET_PLAY_THROW_IN", secondary_time=45)
    checker._restart_at = 0
    state = checker.check_rules(state, sample(1, contacts=(0,), ball_teleported=True))
    assert state.set_play == "SET_PLAY_THROW_IN"
    checker._restart_is_goal = True
    state = checker.check_rules(replace(state, state="STATE_READY", set_play="SET_PLAY_NONE"), sample(2))
    state = checker.check_rules(state, sample(3, contacts=(0,)))
    assert state.state == "STATE_READY" and state.secondary_time > 0


def test_player_mapping_rejects_duplicates_and_respects_teams():
    values = {name: spec.default for name, spec in PARAMETERS.items()}
    values.update(robot_team_mapping='{"0":"home","1":"away","2":"home"}')
    assert RefereeConfig.from_parameters(values).robot_players == {0: 1, 1: 1, 2: 2}
    values["robot_player_mapping"] = '{"0":2}'
    try:
        RefereeConfig.from_parameters(values)
    except ValueError:
        pass
    else:
        raise AssertionError("Duplicate player number accepted")
