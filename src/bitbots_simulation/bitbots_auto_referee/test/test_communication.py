"""Budget accounting and independent receiver liveness without running a simulator."""

from types import SimpleNamespace

from bitbots_auto_referee.adapters.game_controller import encode_game_state
from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.communication import count_team_messages, robot_connections
from bitbots_auto_referee.core.state import MatchState
from game_controller_hsl.data import GameControlDataStruct


def initial_state():
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    return MatchState.initial(config)


def test_budget_is_shared_within_a_team_and_independent_between_teams():
    state = initial_state()
    home, away = [team.team_number for team in state.teams]
    state = count_team_messages(state, {home: 3})
    state = count_team_messages(state, {home: 4, away: 2})
    assert [team.message_budget for team in state.teams] == [11993, 11998]
    packet = GameControlDataStruct.parse(encode_game_state(state, 0))
    assert [team.message_budget for team in packet.teams] == [11993, 11998]


def test_no_received_datagrams_leave_the_state_unchanged():
    state = initial_state()
    assert count_team_messages(state, {}) is state


def test_budget_cannot_underflow_the_unsigned_wire_field():
    state = initial_state()
    state = count_team_messages(state, {state.teams[0].team_number: 13000})
    assert state.teams[0].message_budget == 0
    encode_game_state(state, 0)


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
