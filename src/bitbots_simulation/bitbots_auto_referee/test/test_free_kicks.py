"""Free-kick placement and contact-based indirect goal restrictions."""

from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import RobotContact, RobotMotion, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.check_rules import RuleChecker


def sample(time, ball=(0.0, 0.0, 0.1), touches=(), **changes):
    return SimulationObservation(
        int(time * 1e9), int(time * 1000), ball,
        {0: (1, 1, 0.4), 1: (2, 1, 0.4), 2: (-1, 1, 0.4)}, frozenset(touches), **changes,
    )


def setup():
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    state = replace(MatchState.initial(config), state="STATE_PLAYING", stopped=False)
    commands = Mock()
    ready = Future()
    ready.set_result(TeleportResult(True, "placed"))
    commands.teleport_ball.return_value = ready
    commands.teleport_robot.return_value = ready
    checker = RuleChecker({0: 1, 1: 1, 2: 2}, teleport_commands=commands)
    state = checker.check_rules(state, sample(0))
    return state, checker, commands


def test_freekick_waits_for_actual_placement_and_then_starts_countdown():
    for indirect in (False, True):
        state, checker, commands = setup()
        pending = Future()
        commands.teleport_ball.return_value = pending
        award = checker.award_indirect_free_kick if indirect else checker.award_direct_free_kick
        state = award(state, 2, 1.2, -0.8)
        assert state.stopped and state.secondary_time == 45 and state.kicking_team == 2
        expected = "SET_PLAY_INDIRECT_FREE_KICK" if indirect else "SET_PLAY_DIRECT_FREE_KICK"
        assert state.set_play == expected
        commands.teleport_ball.assert_called_once_with(1.2, -0.8, 0.0)
        state = checker.check_rules(state, sample(1))
        assert state.stopped and state.secondary_time == 45
        pending.set_result(TeleportResult(True, "placed", 2000))
        state = checker.check_rules(state, sample(2, (1.2, -0.8, 0.1), ball_teleported=True))
        assert not state.stopped and state.secondary_time == 45
        state = checker.check_rules(state, sample(3, (1.2, -0.8, 0.1)))
        assert state.secondary_time == 44


def test_indirect_first_touch_clears_set_play_but_not_goal_restriction():
    state, checker, _ = setup()
    state = checker.award_indirect_free_kick(state, 1, 0, 0)
    state = checker.check_rules(state, sample(1, touches=(0,)))
    assert state.set_play == "SET_PLAY_NONE" and state.secondary_time == 0
    state = checker.check_rules(state, sample(2, (-4, 0, 0.1)))
    state = checker.check_rules(state, sample(3, (-5, 0, 0.1)))
    assert checker._pending_outside.kind == "SET_PLAY_GOAL_KICK"
    state = checker.check_rules(state, sample(5, (-5, 0, 0.1)))
    assert state.teams[0].score == 0
    assert state.set_play == "SET_PLAY_GOAL_KICK" and state.kicking_team == 2


def test_another_robot_from_either_team_unlocks_goal():
    for second in (1, 2):
        state, checker, _ = setup()
        state = checker.award_indirect_free_kick(state, 1, 0, 0)
        state = checker.check_rules(state, sample(1, touches=(0,)))
        state = checker.check_rules(state, sample(2, (-4, 0, 0.1), touches=(second,)))
        state = checker.check_rules(state, sample(3, (-5, 0, 0.1)))
        assert checker._pending_outside.kind == "GOAL"
        state = checker.check_rules(state, sample(5, (-5, 0, 0.1)))
        assert state.teams[0].score == 1


def test_throw_in_restriction_survives_timer_expiry_and_blocks_own_goal():
    state, checker, _ = setup()
    checker.last_touch_team_id = 2
    state = checker.check_rules(state, sample(1, (0, 4, 0.1)))
    state = checker.check_rules(state, sample(3, (0, 4, 0.1)))
    assert state.set_play == "SET_PLAY_THROW_IN" and state.kicking_team == 1
    state = checker.check_rules(state, sample(48, (0, 3, 0.1)))
    assert state.set_play == "SET_PLAY_NONE" and checker._indirect_team == 1
    state = checker.check_rules(state, sample(49, (4, 0, 0.1), touches=(0,)))
    state = checker.check_rules(state, sample(50, (5, 0, 0.1)))
    assert checker._pending_outside.kind == "SET_PLAY_CORNER_KICK"
    state = checker.check_rules(state, sample(52, (5, 0, 0.1)))
    assert state.teams[1].score == 0 and state.kicking_team == 2


def test_direct_restart_clears_old_indirect_restriction():
    state, checker, _ = setup()
    state = checker.award_indirect_free_kick(state, 1, 0, 0)
    state = checker.award_direct_free_kick(state, 1, 0, 0)
    state = checker.check_rules(state, sample(1, (-4, 0, 0.1), touches=(0,)))
    state = checker.check_rules(state, sample(2, (-5, 0, 0.1)))
    assert checker._pending_outside.kind == "GOAL"


def test_teleported_robot_contact_does_not_unlock_indirect_goal():
    state, checker, _ = setup()
    state = checker.award_indirect_free_kick(state, 1, 0, 0)
    state = checker.check_rules(state, sample(1, touches=(0,)))
    state = checker.check_rules(state, sample(2, touches=(1,), teleported_robots=frozenset({1})))
    assert checker._indirect_team == 1


def test_pushing_awards_direct_free_kick_at_contact_location():
    state, checker, commands = setup()
    upright = RobotMotion(0, 0, 0, 0, 1, 1)
    state = checker.check_rules(state, sample(0.1, robot_motion={0: upright, 2: upright}))
    contact = RobotContact(0, 2, 30, 0.2, 0, (0.4, 0.7, 0.2))
    state = checker.check_rules(state, sample(
        0.2, robot_motion={0: upright, 2: replace(upright, upright=0.5)}, robot_contacts=(contact,),
    ))
    assert state.teams[0].players[0].penalty == "PENALTY_PUSHING"
    assert state.set_play == "SET_PLAY_DIRECT_FREE_KICK" and state.kicking_team == 2
    assert state.secondary_time == 45
    commands.teleport_ball.assert_called_once_with(0.4, 0.7, 0.0)
    assert state.teams[0].score == state.teams[1].score == 0


def test_later_touch_cannot_change_captured_no_goal_decision():
    state, checker, _ = setup()
    state = checker.award_indirect_free_kick(state, 1, 0, 0)
    state = checker.check_rules(state, sample(1, (-4, 0, 0.1), touches=(0,)))
    state = checker.check_rules(state, sample(2, (-5, 0, 0.1)))
    assert checker._pending_outside.kind == "SET_PLAY_GOAL_KICK"
    state = checker.check_rules(state, sample(3, (-5, 0, 0.1), touches=(2,)))
    state = checker.check_rules(state, sample(4, (-5, 0, 0.1)))
    assert state.teams[0].score == 0 and state.set_play == "SET_PLAY_GOAL_KICK"


def test_two_distinct_robots_in_same_update_unlock_indirect_goal():
    state, checker, _ = setup()
    state = checker.award_indirect_free_kick(state, 1, 0, 0)
    state = checker.check_rules(state, sample(1, touches=(0, 1)))
    assert state.set_play == "SET_PLAY_NONE" and checker._indirect_team is None


def test_second_separate_touch_by_same_robot_unlocks_goal():
    for throw_in in (False, True):
        state, checker, _ = setup()
        if throw_in:
            checker.last_touch_team_id = 2
            state = checker.check_rules(state, sample(1, (0, 4, 0.1)))
            state = checker.check_rules(state, sample(3, (0, 4, 0.1)))
        else:
            state = checker.award_indirect_free_kick(state, 1, 0, 0)
        state = checker.check_rules(state, sample(4, touches=(0,)))
        assert checker._indirect_team == 1
        state = checker.check_rules(state, sample(5))
        state = checker.check_rules(state, sample(6, (-4, 0, 0.1), touches=(0,)))
        assert checker._indirect_team is None
        state = checker.check_rules(state, sample(7, (-5, 0, 0.1)))
        assert checker._pending_outside.kind == "GOAL"
        state = checker.check_rules(state, sample(9, (-5, 0, 0.1)))
        assert state.teams[0].score == 1


def test_continuous_contact_by_same_robot_does_not_unlock_goal():
    state, checker, _ = setup()
    state = checker.award_indirect_free_kick(state, 1, 0, 0)
    for time in (1, 2, 3):
        state = checker.check_rules(state, sample(time, touches=(0,)))
        assert checker._indirect_team == 1
    state = checker.check_rules(state, sample(4, (-4, 0, 0.1)))
    state = checker.check_rules(state, sample(5, (-5, 0, 0.1)))
    assert checker._pending_outside.kind == "SET_PLAY_GOAL_KICK"


def test_double_touch_awards_opponent_indirect_restart_at_ball():
    state, _, commands = setup()
    checker = RuleChecker({0: 1, 1: 1, 2: 1}, teleport_commands=commands)
    state = checker.check_rules(state, sample(0))
    state = checker.award_direct_free_kick(state, 1, 0, 0)
    state = checker.check_rules(state, sample(0.1, touches=(0,), ball_contact_forces={0: 10.0}))
    assert state.set_play == "SET_PLAY_NONE"
    state = checker.check_rules(state, sample(0.2))
    state = checker.check_rules(state, sample(0.3, (0.2, 0.3, 0.1), touches=(0,), ball_contact_forces={0: 10.0}))
    assert state.set_play == "SET_PLAY_INDIRECT_FREE_KICK"
    assert state.kicking_team == 2 and state.secondary_time == 45
    assert all(player.penalty == "PENALTY_NONE" for player in state.teams[0].players)
    commands.teleport_ball.assert_called_with(0.2, 0.3, 0.0)
