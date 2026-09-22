"""Offline protocol regression cases; sockets are replaced with mocks."""

import struct
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from game_controller_hsl.data import GameControlDataStruct

from bitbots_auto_referee.adapters.game_controller import (
    GameControllerUDPAdapter,
    decode_robot_status,
    encode_game_state,
)
from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.state import MatchState

ROSTER_CASES = (
    ("small", "foundation", 4),
    ("small", "advanced", 7),
    ("middle", "foundation", 3),
    ("middle", "advanced", 5),
    ("large", "foundation", 2),
    ("large", "advanced", 3),
)


def config(**overrides):
    values = {name: spec.default for name, spec in PARAMETERS.items()}
    values.update(overrides)
    return RefereeConfig.from_parameters(values)


def return_packet(team=1, player=1, ball_age=2.0):
    return struct.pack("<4s4B6f", b"RGrt", 4, player, team, 1, 1500.0, -500.0, 0.75, ball_age, 250.0, -100.0)


class ProtocolTest(unittest.TestCase):
    def test_match_packet_matches_independent_wire_fixture(self):
        state = MatchState.initial(config())
        header = struct.pack("<4s10Bhh", b"RGme", 20, 255, 4, 0, 1, 0, 0, 0, 1, 1, 600, 0)
        players = bytes([0, 0, 0]) * 4 + bytes([13, 0, 0]) * 16
        home = struct.pack("<6BHH", 1, 0, 0, 1, 0, 0, 0, 0) + players
        away = struct.pack("<6BHH", 2, 1, 1, 1, 0, 0, 0, 0) + players
        self.assertEqual(encode_game_state(state, 255), header + home + away)

    def test_return_packet_converts_units_without_changing_frames(self):
        status = decode_robot_status(return_packet(), 12.0)
        self.assertEqual(status.pose, (1.5, -0.5, 0.75))
        self.assertEqual(status.ball, (0.25, -0.1))
        self.assertEqual(status.received_at, 12.0)
        self.assertTrue(status.fallen)

    def test_return_packet_accepts_unseen_ball(self):
        self.assertEqual(decode_robot_status(return_packet(ball_age=-1.0), 0.0).ball_age, -1.0)

    def test_return_packet_rejects_invalid_data(self):
        packet = return_packet()
        cases = (
            packet[:-1],
            packet + b"extra",
            b"bad!" + packet[4:],
            packet[:4] + b"\xff" + packet[5:],
            return_packet(ball_age=float("nan")),
            return_packet(ball_age=-2.0),
        )
        for invalid in cases:
            with self.subTest(packet=invalid), self.assertRaises(ValueError):
                decode_robot_status(invalid, 0.0)

    def test_roster_exceeding_limit_is_rejected(self):
        with self.assertRaises(ValueError):
            encode_game_state(replace(MatchState.initial(config()), players_per_team=2), 0)

    def test_partial_and_empty_teams_keep_the_match_limit(self):
        for league, lineup, limit in ROSTER_CASES:
            initial = MatchState.initial(config(leagueSize=league, lineup_mode=lineup))
            for home_count, away_count in ((0, 0), (1, 0), (0, 1), (1, 1), (limit, 0)):
                with self.subTest(league=league, lineup=lineup, home=home_count, away=away_count):
                    counts = (home_count, away_count)
                    state = replace(
                        initial,
                        teams=tuple(
                            replace(team, players=team.players[:count], goalkeeper=1 if count else 0)
                            for team, count in zip(initial.teams, counts, strict=True)
                        ),
                    )
                    packet = GameControlDataStruct.parse(encode_game_state(state, 0))
                    self.assertEqual(packet.players_per_team, limit)
                    for team, count in zip(packet.teams, counts, strict=True):
                        self.assertEqual(
                            [player.penalty.intvalue for player in team.players], [0] * count + [13] * (20 - count)
                        )

    def test_derived_rosters_are_transmitted_for_both_teams(self):
        for league, lineup, count in ROSTER_CASES:
            with self.subTest(league=league, lineup=lineup):
                state = MatchState.initial(config(leagueSize=league, lineup_mode=lineup))
                self.assertEqual(state.players_per_team, count)
                self.assertTrue(all(len(team.players) == count for team in state.teams))
                packet = GameControlDataStruct.parse(encode_game_state(state, 0))
                self.assertEqual(packet.players_per_team, count)
                for team in packet.teams:
                    self.assertEqual(len(team.players), 20)
                    self.assertEqual(
                        [player.penalty.intvalue for player in team.players], [0] * count + [13] * (20 - count)
                    )


class ConfigurationTest(unittest.TestCase):
    def test_invalid_startup_settings_are_rejected(self):
        cases = (
            {"home_team_id": 2},
            {"home_team_id": 255},
            {"home_team_id": True},
            {"home_color": "red"},
            {"lineup_mode": "unknown"},
            {"lineup_mode": "formation"},
            {"leagueSize": "unknown"},
            {"players_per_team": 21},
            {"target_port": 0},
            {"return_port": 3838},
            {"target_host": "0.0.0.0"},
            {"target_host": "::1"},
            {"send_rate": float("inf")},
            {"response_timeout": -1.0},
        )
        for invalid in cases:
            with self.subTest(parameters=invalid), self.assertRaises(ValueError):
                config(**invalid)

    def test_advanced_is_a_named_mode(self):
        self.assertEqual(config(lineup_mode="advanced").lineup_mode, "advanced")

    def test_roster_size_is_derived_from_league_and_lineup(self):
        self.assertNotIn("players_per_team", PARAMETERS)
        for league, lineup, count in ROSTER_CASES:
            with self.subTest(league=league, lineup=lineup):
                self.assertEqual(config(leagueSize=league, lineup_mode=lineup).players_per_team, count)


class TransportTest(unittest.TestCase):
    def setUp(self):
        socket_patch = patch("bitbots_auto_referee.adapters.game_controller.socket.socket")
        self.socket_factory = socket_patch.start()
        self.addCleanup(socket_patch.stop)
        self.socket = self.socket_factory.return_value
        self.state = MatchState.initial(config())
        self.adapter = GameControllerUDPAdapter(("127.0.0.1", 3838), ("127.0.0.1", 3939), self.state)
        self.addCleanup(self.adapter.close)

    def test_send_counter_wraps_and_does_not_advance_on_failure(self):
        self.adapter.packet_number = 255
        self.adapter.send_state()
        self.assertEqual(self.adapter.packet_number, 0)
        self.socket.sendto.assert_called_once_with(encode_game_state(self.state, 255), ("127.0.0.1", 3838))
        self.socket.sendto.side_effect = OSError("unreachable")
        with self.assertRaises(OSError):
            self.adapter.send_state()
        self.assertEqual(self.adapter.packet_number, 0)

    def test_bad_replies_do_not_replace_valid_robot_status(self):
        peer = ("127.0.0.1", 3838)
        self.socket.recvfrom.side_effect = [
            (return_packet(), peer),
            (return_packet(team=9), peer),
            (return_packet(player=5), peer),
            (return_packet(), ("127.0.0.2", 3838)),
            (b"truncated", peer),
            BlockingIOError(),
        ]
        replies = self.adapter.receive_responses()
        self.assertEqual(len(replies), 1)
        self.assertEqual(self.adapter.rejected_packets, 4)
        self.assertEqual(self.adapter.latest_responses, {(1, 1): replies[0]})

    def test_receive_work_is_bounded(self):
        self.socket.recvfrom.return_value = (return_packet(), ("127.0.0.1", 3838))
        self.assertEqual(len(self.adapter.receive_responses(limit=3)), 3)
        self.assertEqual(self.socket.recvfrom.call_count, 3)

    def test_send_does_not_require_robot_replies(self):
        self.socket.recvfrom.side_effect = BlockingIOError()
        self.assertEqual(self.adapter.receive_responses(), [])
        self.adapter.send_state()
        self.socket.sendto.assert_called_once_with(encode_game_state(self.state, 0), ("127.0.0.1", 3838))
        self.assertEqual(self.adapter.latest_responses, {})

    def test_one_connected_robot_does_not_require_other_players(self):
        self.socket.recvfrom.side_effect = [
            (return_packet(), ("127.0.0.1", 3838)),
            BlockingIOError(),
        ]
        self.assertEqual(len(self.adapter.receive_responses()), 1)
        self.adapter.send_state()
        self.socket.sendto.assert_called_once_with(encode_game_state(self.state, 0), ("127.0.0.1", 3838))
        self.assertEqual(len(self.adapter.latest_responses), 1)

    def test_derived_roster_controls_reply_acceptance(self):
        for league, lineup, count in ROSTER_CASES:
            with self.subTest(league=league, lineup=lineup):
                self.adapter.set_state(MatchState.initial(config(leagueSize=league, lineup_mode=lineup)))
                rejected_before = self.adapter.rejected_packets
                self.socket.recvfrom.side_effect = [
                    (return_packet(player=count), ("127.0.0.1", 3838)),
                    (return_packet(player=count + 1), ("127.0.0.1", 3838)),
                    BlockingIOError(),
                ]
                replies = self.adapter.receive_responses()
                self.assertEqual([reply.player_number for reply in replies], [count])
                self.assertEqual(self.adapter.rejected_packets, rejected_before + 1)

    def test_invalid_state_preserves_previous_snapshot(self):
        with self.assertRaises(ValueError):
            self.adapter.set_state(replace(self.state, kicking_team=9))
        self.assertIs(self.adapter.state, self.state)

    def test_bind_failure_closes_socket(self):
        failed_socket = Mock()
        failed_socket.bind.side_effect = OSError("address in use")
        self.socket_factory.return_value = failed_socket
        with self.assertRaises(OSError):
            GameControllerUDPAdapter(("127.0.0.1", 3838), ("127.0.0.1", 3939), self.state)
        failed_socket.close.assert_called_once()
