"""Multi-team identities, shared UDP receivers and separate penalty positions."""

import socket
import struct
import sys
from concurrent.futures import Future
from dataclasses import replace
from unittest.mock import Mock, patch

import pytest
from bitbots_auto_referee.adapters.game_controller import GameControllerUDPAdapter, encode_game_state
from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState, PlayerState
from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.motion import MotionRules
from bitbots_auto_referee.rules.outside import FieldGeometry
from builtin_interfaces.msg import Time
from game_controller_hsl.data import GameControlDataStruct
from game_controller_hsl.receiver import GameStateReceiver


def config(**overrides):
    values = {name: spec.default for name, spec in PARAMETERS.items()}
    values.update(overrides)
    return RefereeConfig.from_parameters(values)


def test_missing_multi_team_helper_does_not_guess_a_lineup():
    with patch.dict(sys.modules, {"bitbots_mujoco_sim": None}):
        with pytest.raises(ValueError, match="world configuration"):
            config(num_robots="2:2")


@pytest.mark.parametrize("setup", ["1", "2:2", "3:1", "0:2", "2:0", "7:7"])
def test_lineup_matches_world_configuration(setup):
    world = pytest.importorskip("bitbots_mujoco_sim.world")
    settings = world.compute_game_settings(world.parse_num_robots(setup))
    cfg = config(num_robots=setup, lineup_mode="advanced")
    state = MatchState.initial(cfg)
    for index, robot in enumerate(settings):
        assert cfg.robot_teams[index] == robot["team_id"]
        assert cfg.robot_players[index] == robot["bot_id"]
        assert cfg.robot_roles[index] == robot["role"]
        team = next(team for team in state.teams if team.team_number == robot["team_id"])
        assert (team.goalkeeper == robot["bot_id"]) == (robot["role"] == "goalie")
    assert cfg.home_defends_negative_x
    assert state.teams[0].field_player_color == "BLUE"
    assert state.teams[1].field_player_color == "RED"
    assert state.teams[0].goalkeeper_color == "YELLOW"
    assert state.teams[1].goalkeeper_color == "GREEN"
    packet = GameControlDataStruct.parse(encode_game_state(state, 0))
    assert [team.goalkeeper for team in packet.teams] == [team.goalkeeper for team in state.teams]


def test_simulator_lineup_is_authoritative():
    world = pytest.importorskip("bitbots_mujoco_sim.world")
    cfg = config(
        num_robots="2:2",
        home_team_id=10,
        away_team_id=11,
        home_defends_negative_x=False,
        robot_team_mapping="{}",
        robot_player_mapping="{}",
        home_color="white",
    )
    assert cfg.robot_teams == {
        0: world.BASE_TEAM_ID,
        1: world.BASE_TEAM_ID,
        2: world.BASE_TEAM_ID + 1,
        3: world.BASE_TEAM_ID + 1,
    }
    assert cfg.robot_players == {0: 1, 1: 2, 2: 1, 3: 2}
    assert cfg.home_color == "blue" and cfg.home_defends_negative_x


@pytest.mark.parametrize("setup", ["1:1:1", "5:1", "-1:2", "0:0"])
def test_invalid_or_oversized_match_is_rejected(setup):
    pytest.importorskip("bitbots_mujoco_sim.world")
    with pytest.raises(ValueError):
        config(num_robots=setup)


def test_shared_packet_produces_individual_robot_states_for_both_teams():
    state = MatchState.initial(config(home_goalkeeper=2, away_goalkeeper=2))
    home, away = state.teams
    home = replace(home, score=3, players=(PlayerState(), PlayerState("PENALTY_PUSHING", 45), *home.players[2:]))
    away = replace(away, score=1)
    packet = GameControlDataStruct.parse(encode_game_state(replace(state, teams=(home, away)), 0))
    for team in (home, away):
        for player in (1, 2):
            receiver = Mock(team_number=team.team_number, player_number=player)
            receiver.get_clock.return_value.now.return_value.to_msg.return_value = Time()
            message = GameStateReceiver.build_game_state_msg(receiver, packet)
            assert message.own_score == team.score
            assert message.penalized == (team == home and player == 2)
            assert message.team_mates_with_penalty[:2] == ([False, True] if team == home else [False, False])


@pytest.mark.parametrize("first_half", [True, False])
@pytest.mark.parametrize("side", [-1, 1])
def test_simultaneous_penalties_have_separate_slots_in_own_half(first_half, side):
    cfg = config()
    state = replace(MatchState.initial(cfg), state="STATE_PLAYING", stopped=False, first_half=first_half)
    teams = {0: cfg.home_team_id, 1: cfg.home_team_id, 2: cfg.away_team_id, 3: cfg.away_team_id}
    players = {0: 1, 1: 2, 2: 1, 3: 2}
    teleport = Mock()
    completed = Future()
    completed.set_result(TeleportResult(True, "placed"))
    teleport.return_value = completed
    rules = MotionRules(teams, players, FieldGeometry.from_config(cfg), teleport, Mock())
    observation = SimulationObservation(0, 0, (0, 0, 0.1), {robot: (0, side, 0.4) for robot in teams}, frozenset())
    for robot in teams:
        state = rules.penalize(state, observation, robot, "PENALTY_PUSHING")
    positions = [call.args[1:3] for call in teleport.call_args_list]
    assert len(set(positions)) == len(teams)
    for call in teleport.call_args_list:
        robot, x, y, yaw = call.args
        home = teams[robot] == cfg.home_team_id
        assert (x < 0) == (home == first_half)
        assert 0 < abs(x) < cfg.field_length / 2
        assert y * side > cfg.field_width / 2
        assert yaw * side < 0
    assert abs(positions[0][0] - positions[1][0]) > 0.5


@pytest.mark.skipif(sys.platform != "linux", reason="The simulator targets Linux UDP broadcast semantics")
def test_broadcast_reaches_all_robot_receivers_and_collects_both_teams():
    cfg = config()
    state = MatchState.initial(cfg)
    receivers = []
    adapter = None
    try:
        port = 0
        for _ in range(4):
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receivers.append(receiver)
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            receiver.bind(("0.0.0.0", port))
            receiver.settimeout(1.0)
            port = receiver.getsockname()[1]
        adapter = GameControllerUDPAdapter((cfg.target_host, port), (cfg.bind_host, 0), state, broadcast=True)
        adapter.send_state()
        roster = [(cfg.home_team_id, 1), (cfg.home_team_id, 2), (cfg.away_team_id, 1), (cfg.away_team_id, 2)]
        packets = []
        for receiver, (team, player) in zip(receivers, roster, strict=True):
            packet, _ = receiver.recvfrom(GameControlDataStruct.sizeof() + 1)
            packets.append(packet)
            reply = struct.pack("<4s4B6f", b"RGrt", 4, player, team, 0, 0, 0, 0, -1, 0, 0)
            receiver.sendto(reply, adapter._socket.getsockname())
        assert packets == [packets[0]] * len(receivers)
        assert {(reply.team_number, reply.player_number) for reply in adapter.receive_responses()} == set(roster)
        assert set(adapter.latest_responses) == set(roster)
    finally:
        if adapter is not None:
            adapter.close()
        for receiver in receivers:
            receiver.close()
