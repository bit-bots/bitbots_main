import pytest

from bitbots_mujoco_sim.sim_gamestate_gui import SimGame, build_game_state
from bitbots_mujoco_sim.world import compute_game_settings, parse_num_robots
from game_controller_hsl_interfaces.msg import GameState


def test_game_state_is_flipped_for_each_team() -> None:
    settings = compute_game_settings((2, 2))
    game = SimGame(main_state=GameState.STATE_PLAYING, kicking_team=7, scores={6: 2, 7: 1})

    team_one = build_game_state(game, settings[0], (2, 2))
    team_two = build_game_state(game, settings[2], (2, 2))

    assert team_one.kicking_team == team_two.kicking_team == 7
    assert (team_one.own_score, team_one.rival_score) == (2, 1)
    assert (team_two.own_score, team_two.rival_score) == (1, 2)
    assert (team_one.own_player_color, team_one.rival_player_color) == (GameState.TEAM_BLUE, GameState.TEAM_RED)
    assert (team_two.own_player_color, team_two.rival_player_color) == (GameState.TEAM_RED, GameState.TEAM_BLUE)
    assert (team_one.own_goalie_color, team_one.rival_goalie_color) == (GameState.TEAM_YELLOW, GameState.TEAM_GREEN)
    assert (team_two.own_goalie_color, team_two.rival_goalie_color) == (GameState.TEAM_GREEN, GameState.TEAM_YELLOW)


def test_single_team_setup_has_default_rival() -> None:
    settings = compute_game_settings((2, 0))
    game = SimGame(scores={6: 0, 7: 0})

    msg = build_game_state(game, settings[0], (2, 0))

    assert msg.kicking_team == 6
    assert (msg.own_score, msg.rival_score) == (0, 0)
    assert (msg.own_player_color, msg.rival_player_color) == (GameState.TEAM_BLUE, GameState.TEAM_RED)


def test_game_state_supports_empty_first_team() -> None:
    team_sizes = (0, 2)
    settings = compute_game_settings(team_sizes)
    game = SimGame(kicking_team=7, scores={6: 0, 7: 1})

    msg = build_game_state(game, settings[0], team_sizes)

    assert msg.players_per_team == 2
    assert (msg.own_score, msg.rival_score) == (1, 0)
    assert (msg.own_player_color, msg.rival_player_color) == (GameState.TEAM_RED, GameState.TEAM_BLUE)


def test_parse_num_robots_returns_two_team_sizes() -> None:
    assert parse_num_robots("3") == (3, 0)
    assert parse_num_robots("2:1") == (2, 1)


@pytest.mark.parametrize("spec", ["1:2:3", "0:0", "-1:2"])
def test_parse_num_robots_rejects_invalid_setups(spec: str) -> None:
    with pytest.raises(ValueError):
        parse_num_robots(spec)