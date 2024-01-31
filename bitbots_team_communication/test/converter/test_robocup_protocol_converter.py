from unittest.mock import MagicMock

import pytest
from soccer_vision_attribute_msgs.msg import Robot as RobotAttributes

import bitbots_team_communication.robocup_extension_pb2 as Proto  # noqa: N812
from bitbots_team_communication.converter.robocup_protocol_converter import RobocupProtocolConverter, TeamColor

own_team_color = TeamColor.BLUE


def test_setup_of_mappings(snapshot):
    converter = protocol_converter()
    assert converter.role_mapping == snapshot
    assert converter.action_mapping == snapshot
    assert converter.side_mapping == snapshot


def test_setup_of_team_color_mapping(snapshot):
    converter = protocol_converter()
    assert converter.state_to_proto_team_mapping[RobotAttributes.TEAM_OWN] == Proto.Team.BLUE
    assert converter.state_to_proto_team_mapping[RobotAttributes.TEAM_OPPONENT] == Proto.Team.RED
    assert converter.state_to_proto_team_mapping[RobotAttributes.TEAM_UNKNOWN] == Proto.Team.UNKNOWN_TEAM
    assert converter.proto_to_team_data_team_mapping == snapshot


def test_setup_of_to_message_converter(snapshot, to_message_mock):
    protocol_converter()

    to_message_mock.assert_called_once()
    args = to_message_mock.mock_calls[0].kwargs
    assert args == snapshot


def test_setup_of_from_message_converter(snapshot, from_message_mock):
    protocol_converter()

    from_message_mock.assert_called_once()
    args = from_message_mock.mock_calls[0].kwargs
    assert args == snapshot


def test_maps_convert_functions(to_message_mock, from_message_mock):
    to_message_mock.return_value.convert.return_value = "message"
    from_message_mock.return_value.convert.return_value = "team_data"

    converter = protocol_converter()

    assert converter.convert_to_message() == "message"
    assert converter.convert_from_message() == "team_data"


def protocol_converter() -> RobocupProtocolConverter:
    return RobocupProtocolConverter(own_team_color)


@pytest.fixture
def to_message_mock(mocker) -> MagicMock:
    return mocker.patch("bitbots_team_communication.converter.robocup_protocol_converter.StateToMessageConverter")


@pytest.fixture
def from_message_mock(mocker) -> MagicMock:
    return mocker.patch("bitbots_team_communication.converter.robocup_protocol_converter.MessageToTeamDataConverter")
