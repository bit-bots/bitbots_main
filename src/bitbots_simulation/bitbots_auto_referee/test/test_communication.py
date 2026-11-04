"""Budget accounting and independent receiver liveness without running a simulator."""

from types import SimpleNamespace

import pytest
from bitbots_auto_referee.adapters.game_controller import encode_game_state
from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.communication import MessageBudget, robot_connections
from bitbots_auto_referee.core.state import MatchState
from game_controller_hsl.data import GameControlDataStruct


def initial_state():
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    return MatchState.initial(config)


def test_budget_is_shared_within_a_team_and_independent_between_teams():
    budget = MessageBudget()
    state = initial_state()
    assert [team.message_budget for team in state.teams] == [12000, 12000]
    home, away = [team.team_number for team in state.teams]
    state = budget.update(state, home, 1, "first", 3)
    state = budget.update(state, home, 2, "second", 4)
    state = budget.update(state, away, 1, "third", 2)
    assert [team.message_budget for team in state.teams] == [11993, 11998]
    packet = GameControlDataStruct.parse(encode_game_state(state, 0))
    assert [team.message_budget for team in packet.teams] == [11993, 11998]


def test_duplicate_out_of_order_and_skipped_reports_do_not_corrupt_budget():
    budget = MessageBudget()
    state = initial_state()
    team = state.teams[0].team_number
    state = budget.update(state, team, 1, "sender", 10)
    assert budget.update(state, team, 1, "sender", 10) is state
    assert budget.update(state, team, 1, "sender", 4) is state
    state = budget.update(state, team, 1, "sender", 23)
    assert state.teams[0].message_budget == 11977


def test_sender_restart_and_delayed_final_report_from_previous_session():
    budget = MessageBudget()
    state = initial_state()
    team = state.teams[0].team_number
    state = budget.update(state, team, 1, "old", 8)
    state = budget.update(state, team, 1, "new", 2)
    assert budget.update(state, team, 1, "old", 8) is state
    state = budget.update(state, team, 1, "old", 9)
    state = budget.update(state, team, 1, "new", 5)
    assert state.teams[0].message_budget == 11986


def test_budget_cannot_underflow_the_unsigned_wire_field():
    state = initial_state()
    state = MessageBudget().update(state, state.teams[0].team_number, 1, "sender", 13000)
    assert state.teams[0].message_budget == 0
    encode_game_state(state, 0)


@pytest.mark.parametrize("session,count", [("", 1), ("sender", -1), ("sender", True), ("sender", 1.5)])
def test_invalid_send_reports_are_rejected(session, count):
    state = initial_state()
    with pytest.raises(ValueError):
        MessageBudget().update(state, state.teams[0].team_number, 1, session, count)


def test_connection_status_does_not_hide_missing_robots_or_an_entire_team():
    expected = {(6, 1), (6, 2), (7, 1)}
    responses = {(6, 1): SimpleNamespace(received_at=9.0), (7, 1): SimpleNamespace(received_at=1.0)}
    assert robot_connections(expected, responses, 10.0, 0.0, 5.0) == {
        (6, 1): True,
        (6, 2): False,
        (7, 1): False,
    }
    responses[(7, 1)] = SimpleNamespace(received_at=10.0)
    assert robot_connections(expected, responses, 10.0, 0.0, 5.0)[(7, 1)] is True


def test_unseen_robot_waits_only_during_startup_grace_period():
    assert robot_connections({(6, 1)}, {}, 3.0, 0.0, 5.0) == {(6, 1): None}
    assert robot_connections({(6, 1)}, {}, 5.0, 0.0, 5.0) == {(6, 1): False}
