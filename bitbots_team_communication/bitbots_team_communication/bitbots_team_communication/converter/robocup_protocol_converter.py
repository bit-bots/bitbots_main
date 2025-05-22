from enum import IntEnum

from soccer_vision_attribute_msgs.msg import Robot as RobotAttributes

import bitbots_team_communication.robocup_extension_pb2 as Proto  # noqa: N812
from bitbots_msgs.msg import RobotRelative, Strategy
from bitbots_team_communication.converter.message_to_team_data_converter import MessageToTeamDataConverter
from bitbots_team_communication.converter.state_to_message_converter import StateToMessageConverter


class TeamColor(IntEnum):
    BLUE = 0
    RED = 1


class RobocupProtocolConverter:
    def __init__(self, own_team_color: TeamColor):
        self.role_mapping = (
            (Proto.ROLE_UNDEFINED, Strategy.ROLE_UNDEFINED),
            (Proto.ROLE_IDLING, Strategy.ROLE_IDLING),
            (Proto.ROLE_OTHER, Strategy.ROLE_OTHER),
            (Proto.ROLE_STRIKER, Strategy.ROLE_STRIKER),
            (Proto.ROLE_SUPPORTER, Strategy.ROLE_SUPPORTER),
            (Proto.ROLE_DEFENDER, Strategy.ROLE_DEFENDER),
            (Proto.ROLE_GOALIE, Strategy.ROLE_GOALIE),
        )
        self.action_mapping = (
            (Proto.ACTION_UNDEFINED, Strategy.ACTION_UNDEFINED),
            (Proto.ACTION_POSITIONING, Strategy.ACTION_POSITIONING),
            (Proto.ACTION_GOING_TO_BALL, Strategy.ACTION_GOING_TO_BALL),
            (Proto.ACTION_TRYING_TO_SCORE, Strategy.ACTION_TRYING_TO_SCORE),
            (Proto.ACTION_WAITING, Strategy.ACTION_WAITING),
            (Proto.ACTION_KICKING, Strategy.ACTION_KICKING),
            (Proto.ACTION_SEARCHING, Strategy.ACTION_SEARCHING),
            (Proto.ACTION_LOCALIZING, Strategy.ACTION_LOCALIZING),
        )
        self.side_mapping = (
            (Proto.SIDE_UNDEFINED, Strategy.SIDE_UNDEFINED),
            (Proto.SIDE_LEFT, Strategy.SIDE_LEFT),
            (Proto.SIDE_MIDDLE, Strategy.SIDE_MIDDLE),
            (Proto.SIDE_RIGHT, Strategy.SIDE_RIGHT),
        )

        self.proto_to_team_data_team_mapping = {
            Proto.UNKNOWN_TEAM: RobotRelative.ROBOT_UNDEFINED,
            Proto.BLUE: RobotRelative.ROBOT_BLUE,
            Proto.RED: RobotRelative.ROBOT_RED,
        }
        self.state_to_proto_team_mapping = {
            RobotAttributes.TEAM_OWN: Proto.RED if own_team_color == TeamColor.RED else Proto.BLUE,
            RobotAttributes.TEAM_OPPONENT: Proto.BLUE if own_team_color == TeamColor.RED else Proto.RED,
            RobotAttributes.TEAM_UNKNOWN: Proto.UNKNOWN_TEAM,
        }

        proto_to_team_data_mappings = {
            "team_mapping": self.proto_to_team_data_team_mapping,
            "role_mapping": dict(self.role_mapping),
            "action_mapping": dict(self.action_mapping),
            "side_mapping": dict(self.side_mapping),
        }

        def reverse_mapping(mapping):
            return dict((b, a) for a, b in mapping)

        state_to_proto_mappings = {
            "team_mapping": self.state_to_proto_team_mapping,
            "role_mapping": reverse_mapping(self.role_mapping),
            "action_mapping": reverse_mapping(self.action_mapping),
            "side_mapping": reverse_mapping(self.side_mapping),
        }

        self.convert_from_message = MessageToTeamDataConverter(**proto_to_team_data_mappings).convert
        self.convert_to_message = StateToMessageConverter(**state_to_proto_mappings).convert
