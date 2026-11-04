import socket
import threading
from unittest.mock import Mock

import pytest
from bitbots_mujoco_sim.sim_gamestate_gui import (
    SimGame,
    TeamCommBudgetListener,
    build_game_state,
    consume_message_budget,
    score_goal,
)
from bitbots_mujoco_sim.world import compute_game_settings, parse_num_robots
from game_controller_hsl_interfaces.msg import GameState


def test_game_state_is_flipped_for_each_team() -> None:
    settings = compute_game_settings((2, 2))
    game = SimGame(
        main_state=GameState.STATE_PLAYING,
        kicking_team=7,
        scores={6: 2, 7: 1},
        message_budgets={6: 20, 7: 18},
    )

    team_one = build_game_state(game, settings[0], (2, 2))
    team_two = build_game_state(game, settings[2], (2, 2))

    assert team_one.kicking_team == team_two.kicking_team == 7
    assert (team_one.own_score, team_one.rival_score) == (2, 1)
    assert (team_two.own_score, team_two.rival_score) == (1, 2)
    assert (team_one.own_player_color, team_one.rival_player_color) == (GameState.TEAM_BLUE, GameState.TEAM_RED)
    assert (team_two.own_player_color, team_two.rival_player_color) == (GameState.TEAM_RED, GameState.TEAM_BLUE)
    assert (team_one.own_goalie_color, team_one.rival_goalie_color) == (GameState.TEAM_YELLOW, GameState.TEAM_GREEN)
    assert (team_two.own_goalie_color, team_two.rival_goalie_color) == (GameState.TEAM_GREEN, GameState.TEAM_YELLOW)
    assert (team_one.message_budget, team_two.message_budget) == (20, 18)


def test_single_team_setup_has_default_rival() -> None:
    settings = compute_game_settings((2, 0))
    game = SimGame(scores={6: 0, 7: 0}, message_budgets={6: 10, 7: 10})

    msg = build_game_state(game, settings[0], (2, 0))

    assert msg.kicking_team == 6
    assert (msg.own_score, msg.rival_score) == (0, 0)
    assert (msg.own_player_color, msg.rival_player_color) == (GameState.TEAM_BLUE, GameState.TEAM_RED)
    assert msg.message_budget == 10


def test_game_state_supports_empty_first_team() -> None:
    team_sizes = (0, 2)
    settings = compute_game_settings(team_sizes)
    game = SimGame(kicking_team=7, scores={6: 0, 7: 1}, message_budgets={6: 5, 7: 3})

    msg = build_game_state(game, settings[0], team_sizes)

    assert msg.players_per_team == 2
    assert (msg.own_score, msg.rival_score) == (1, 0)
    assert (msg.own_player_color, msg.rival_player_color) == (GameState.TEAM_RED, GameState.TEAM_BLUE)


@pytest.mark.parametrize(("scoring_team_id", "conceding_team_id"), [(6, 7), (7, 6)])
def test_scoring_goal_resets_game_to_ready(scoring_team_id: int, conceding_team_id: int) -> None:
    game = SimGame(
        main_state=GameState.STATE_PLAYING,
        kicking_team=scoring_team_id,
        scores={6: 0, 7: 0},
        message_budgets={6: 0, 7: 0},
    )

    score_goal(game, scoring_team_id)

    assert game.scores[scoring_team_id] == 1
    assert game.scores[conceding_team_id] == 0
    assert game.main_state == GameState.STATE_READY
    assert game.kicking_team == conceding_team_id


def test_scoring_goal_rejects_unknown_team() -> None:
    game = SimGame(scores={6: 0, 7: 0}, message_budgets={6: 0, 7: 0})

    with pytest.raises(ValueError, match="not part of this game"):
        score_goal(game, 8)


def test_consuming_message_budget_tracks_sent_and_remaining_messages() -> None:
    game = SimGame(message_budgets={6: 5, 7: 3})

    consume_message_budget(game, 6, 2)
    consume_message_budget(game, 7, 5)

    assert game.sent_messages == {6: 2, 7: 5}
    assert game.message_budgets == {6: 3, 7: 0}


def test_consuming_message_budget_rejects_unknown_team_and_negative_count() -> None:
    game = SimGame(message_budgets={6: 5, 7: 3})

    with pytest.raises(ValueError, match="no message budget"):
        consume_message_budget(game, 8, 1)
    with pytest.raises(ValueError, match="cannot be negative"):
        consume_message_budget(game, 6, -1)


def test_team_comm_budget_listener_counts_team_packets() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        base_port = probe.getsockname()[1] - 70

    received_counts: dict[int, int] = {}
    counts_lock = threading.Lock()
    received_both_teams = threading.Event()

    def on_messages(team_id: int, count: int) -> None:
        with counts_lock:
            received_counts[team_id] = received_counts.get(team_id, 0) + count
            if received_counts == {6: 1, 7: 1}:
                received_both_teams.set()

    listener = TeamCommBudgetListener(Mock(), base_port, on_messages)
    assert listener.enabled
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(b"team six", ("127.0.0.1", base_port + 60))
            sender.sendto(b"team seven", ("127.0.0.1", base_port + 70))

        assert received_both_teams.wait(timeout=2.0)
        assert received_counts == {6: 1, 7: 1}
    finally:
        listener.close()


def test_parse_num_robots_returns_two_team_sizes() -> None:
    assert parse_num_robots("3") == (3, 0)
    assert parse_num_robots("2:1") == (2, 1)


@pytest.mark.parametrize("spec", ["1:2:3", "0:0", "-1:2"])
def test_parse_num_robots_rejects_invalid_setups(spec: str) -> None:
    with pytest.raises(ValueError):
        parse_num_robots(spec)
