"""Count a logical team broadcast once, including a partially successful fanout."""

from ipaddress import IPv4Address
from unittest.mock import Mock

import pytest

from bitbots_team_communication.bitbots_team_communication import TeamCommunication
from bitbots_team_communication.communication import SocketCommunication


def communication():
    return Mock(target_ports=[4061, 4062, 4063, 4064], target_ip=IPv4Address("127.0.0.1"))


def test_local_fanout_consumes_one_message():
    socket = communication()
    sent = Mock()
    SocketCommunication.send_message(socket, b"message", on_sent=sent)
    assert socket.socket.sendto.call_count == 4
    sent.assert_called_once_with()


def test_no_successful_datagram_does_not_consume_budget():
    socket = communication()
    socket.socket.sendto.side_effect = OSError("unreachable")
    sent = Mock()
    with pytest.raises(OSError):
        SocketCommunication.send_message(socket, b"message", on_sent=sent)
    sent.assert_not_called()


def test_partial_fanout_is_still_counted_once():
    socket = communication()
    socket.socket.sendto.side_effect = [7, OSError("unreachable")]
    sent = Mock()
    with pytest.raises(OSError):
        SocketCommunication.send_message(socket, b"message", on_sent=sent)
    sent.assert_called_once_with()


def test_simulation_can_use_the_last_message_but_hardware_keeps_its_reserve():
    communication = Mock(gamestate=Mock(penalized=False, message_budget=1))
    communication.node.get_parameter.return_value.value = True
    assert TeamCommunication.is_robot_allowed_to_send_message(communication)
    communication.gamestate.message_budget = 0
    assert not TeamCommunication.is_robot_allowed_to_send_message(communication)
    communication.node.get_parameter.return_value.value = False
    communication.gamestate.message_budget = 39
    assert not TeamCommunication.is_robot_allowed_to_send_message(communication)
    communication.gamestate.message_budget = 40
    assert TeamCommunication.is_robot_allowed_to_send_message(communication)


def test_oversized_or_disallowed_messages_are_not_reported_as_sent():
    communication = Mock(max_message_size=1, lifetime=1)
    communication.is_robot_allowed_to_send_message.return_value = False
    TeamCommunication.send_message(communication)
    communication.socket_communication.send_message.assert_not_called()
    communication.is_robot_allowed_to_send_message.return_value = True
    communication.protocol_converter.convert_to_message.return_value.SerializeToString.return_value = b"too long"
    TeamCommunication.send_message(communication)
    communication.socket_communication.send_message.assert_not_called()
