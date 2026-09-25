"""Pushing attribution, exemptions, effect windows and uninterrupted contact timing."""

import math
from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import RobotContact, RobotMotion, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.check_rules import RuleChecker
from bitbots_auto_referee.rules.pushing import PushingConfig

UP = RobotMotion(0, 0, 0, 0, 1, 1)


def setup(config=None):
    values = RefereeConfig.from_parameters({key: spec.default for key, spec in PARAMETERS.items()})
    state = replace(MatchState.initial(values), state="STATE_PLAYING", stopped=False)
    commands = Mock()
    completed = Future()
    completed.set_result(TeleportResult(True, "placed"))
    commands.teleport_robot.return_value = completed
    commands.teleport_ball.return_value = completed
    return state, RuleChecker({0: 1, 1: 2}, teleport_commands=commands, pushing_config=config), commands


def sample(time, contacts=(), victim=UP, **changes):
    return SimulationObservation(
        int(time * 1e9),
        int(time * 1000),
        changes.pop("ball_position", (3, 2, 0.1)),
        {0: (-0.25, 0, 0.4), 1: (0.25, 0, 0.4)},
        frozenset(),
        robot_motion={0: UP, 1: victim},
        robot_yaws={0: 0, 1: math.pi},
        robot_contacts=contacts,
        **changes,
    )


def penalty(state):
    return state.teams[0].players[0].penalty


def test_single_contact_requires_destabilization_and_correct_actor():
    state, checker, commands = setup()
    state = checker.check_rules(state, sample(0))
    contact = RobotContact(0, 1, 30, 0.2, 0)
    state = checker.check_rules(state, sample(0.1, (contact,)))
    assert penalty(state) == "PENALTY_NONE"
    state = checker.check_rules(state, sample(0.2, victim=replace(UP, upright=0.5)))
    assert penalty(state) == "PENALTY_PUSHING"
    assert state.teams[0].players[0].secs_till_unpenalized == 45
    assert state.teams[1].players[0].penalty == "PENALTY_NONE"
    commands.teleport_robot.assert_called_once()
    assert commands.teleport_robot.call_args.args[0] == 0


def test_sustained_contact_and_gap_reset():
    state, checker, _ = setup()
    contact = RobotContact(0, 1, 3, 0.2, 0)
    for time in (0, 4.9):
        state = checker.check_rules(state, sample(time, (contact,)))
    assert penalty(state) == "PENALTY_NONE"
    state = checker.check_rules(state, sample(5, (contact,)))
    assert penalty(state) == "PENALTY_PUSHING"
    state, checker, _ = setup()
    state = checker.check_rules(state, sample(0, (contact,)))
    state = checker.check_rules(state, sample(4))
    for time in (4.1, 5.1):
        state = checker.check_rules(state, sample(time, (contact,)))
    assert penalty(state) == "PENALTY_NONE"


def test_mutual_pushing_and_central_ball_duels_are_exempt():
    for mutual in (True, False):
        state, checker, commands = setup()
        contact = RobotContact(0, 1, 30, 0.2, 0.2 if mutual else 0)
        for time in (0, 1, 6):
            state = checker.check_rules(
                state, sample(time, (contact,), ball_position=(3, 2, 0.1) if mutual else (0, 0, 0.1))
            )
        assert penalty(state) == "PENALTY_NONE"
        commands.teleport_robot.assert_not_called()


def test_force_threshold_and_expired_effect_window():
    for config, delay in ((PushingConfig(force_threshold=100), 0.2), (PushingConfig(), 1.0)):
        state, checker, _ = setup(config)
        state = checker.check_rules(state, sample(0))
        state = checker.check_rules(state, sample(0.1, (RobotContact(0, 1, 30, 0.2, 0),)))
        state = checker.check_rules(state, sample(delay, victim=replace(UP, upright=0.4)))
        assert penalty(state) == "PENALTY_NONE"


def test_teleport_cancels_contact_episode():
    state, checker, _ = setup()
    contact = RobotContact(0, 1, 3, 0.2, 0)
    state = checker.check_rules(state, sample(0, (contact,)))
    state = checker.check_rules(state, sample(4, (contact,), teleported_robots=frozenset({1})))
    state = checker.check_rules(state, sample(5, (contact,)))
    assert penalty(state) == "PENALTY_NONE"
