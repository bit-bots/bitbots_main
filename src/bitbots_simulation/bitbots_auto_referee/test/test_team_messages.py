"""Count actual loopback datagrams independently of robot code and ROS domains."""

import ast
import socket
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch

import pytest
from bitbots_auto_referee.adapters.simulation import team_messages
from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.node import AutoReferee


def test_counting_ports_do_not_overlap_player_ports_across_teams():
    launch = (
        Path(__file__).resolve().parents[3]
        / "bitbots_team_communication/bitbots_team_communication/launch/team_comm.launch"
    )
    simulation_ports = ET.parse(launch).getroot().find("node[@if]/param[@name='local_target_ports']")
    destinations = ast.literal_eval(simulation_ports.attrib["value"])
    players, counter = destinations[:-1], destinations[-1]
    assert len(players) == 7
    occupied = set()
    for team in range(1, 255):
        ports = {base + 10 * team for base in destinations}
        assert len(ports) == len(destinations)
        assert occupied.isdisjoint(ports)
        assert team_messages.team_message_port(team) == counter + 10 * team
        assert team_messages.team_message_port(team) not in {base + 10 * team for base in players}
        occupied.update(ports)


def test_udp_copies_are_counted_per_team_without_payload_deduplication(monkeypatch):
    monkeypatch.setattr(team_messages, "team_message_port", lambda team: 0)
    receiver = team_messages.TeamMessageReceiver((6, 7))
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            home = receiver._sockets[6].getsockname()
            away = receiver._sockets[7].getsockname()
            sender.sendto(b"same payload", home)
            sender.sendto(b"same payload", home)
            sender.sendto(b"other team", away)
        assert receiver.receive_counts() == {6: 2, 7: 1}
        assert receiver.receive_counts() == {}
    finally:
        receiver.close()
    assert receiver._sockets == {}


def test_only_the_additional_fanout_destination_consumes_budget(monkeypatch):
    monkeypatch.setattr(team_messages, "team_message_port", lambda team: 0)
    receiver = team_messages.TeamMessageReceiver((6,))
    players = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(7)]
    try:
        for player in players:
            player.bind(("127.0.0.1", 0))
        destinations = [player.getsockname() for player in players] + [receiver._sockets[6].getsockname()]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for target in destinations:
                sender.sendto(b"team message", target)
        assert receiver.receive_counts() == {6: 1}
    finally:
        receiver.close()
        for player in players:
            player.close()


def test_failed_bind_closes_previously_opened_sockets():
    first, second = Mock(), Mock()
    second.bind.side_effect = OSError("address in use")
    with patch.object(team_messages.socket, "socket", side_effect=[first, second]):
        with pytest.raises(OSError, match="address in use"):
            team_messages.TeamMessageReceiver((6, 7))
    first.close.assert_called_once_with()
    second.close.assert_called_once_with()


def test_udp_counts_update_and_send_the_gamecontroller_budget():
    config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
    state = MatchState.initial(config)
    node = Mock()
    node.adapter.state = state
    node._team_messages.receive_counts.return_value = {config.home_team_id: 3, config.away_team_id: 2}
    AutoReferee._receive_team_messages(node)
    updated = node.adapter.set_state.call_args.args[0]
    assert [team.message_budget for team in updated.teams] == [
        state.teams[0].message_budget - 3,
        state.teams[1].message_budget - 2,
    ]
    node._send.assert_called_once_with()
