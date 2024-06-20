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
            (Proto.Role.ROLE_UNDEFINED, Strategy.ROLE_UNDEFINED),
            (Proto.Role.ROLE_IDLING, Strategy.ROLE_IDLING),
            (Proto.Role.ROLE_OTHER, Strategy.ROLE_OTHER),
            (Proto.Role.ROLE_STRIKER, Strategy.ROLE_STRIKER),
            (Proto.Role.ROLE_SUPPORTER, Strategy.ROLE_SUPPORTER),
            (Proto.Role.ROLE_DEFENDER, Strategy.ROLE_DEFENDER),
            (Proto.Role.ROLE_GOALIE, Strategy.ROLE_GOALIE),
        )
        self.action_mapping = (
            (Proto.Action.ACTION_UNDEFINED, Strategy.ACTION_UNDEFINED),
            (Proto.Action.ACTION_POSITIONING, Strategy.ACTION_POSITIONING),
            (Proto.Action.ACTION_GOING_TO_BALL, Strategy.ACTION_GOING_TO_BALL),
            (Proto.Action.ACTION_TRYING_TO_SCORE, Strategy.ACTION_TRYING_TO_SCORE),
            (Proto.Action.ACTION_WAITING, Strategy.ACTION_WAITING),
            (Proto.Action.ACTION_KICKING, Strategy.ACTION_KICKING),
            (Proto.Action.ACTION_SEARCHING, Strategy.ACTION_SEARCHING),
            (Proto.Action.ACTION_LOCALIZING, Strategy.ACTION_LOCALIZING),
        )
        self.side_mapping = (
            (Proto.OffensiveSide.SIDE_UNDEFINED, Strategy.SIDE_UNDEFINED),
            (Proto.OffensiveSide.SIDE_LEFT, Strategy.SIDE_LEFT),
            (Proto.OffensiveSide.SIDE_MIDDLE, Strategy.SIDE_MIDDLE),
            (Proto.OffensiveSide.SIDE_RIGHT, Strategy.SIDE_RIGHT),
        )

        self.proto_to_team_data_team_mapping = {
            Proto.Team.UNKNOWN_TEAM: RobotRelative.ROBOT_UNDEFINED,
            Proto.Team.BLUE: RobotRelative.ROBOT_BLUE,
            Proto.Team.RED: RobotRelative.ROBOT_RED,
        }
        self.state_to_proto_team_mapping = {
            RobotAttributes.TEAM_OWN: Proto.Team.RED if own_team_color == TeamColor.RED else Proto.Team.BLUE,
            RobotAttributes.TEAM_OPPONENT: Proto.Team.BLUE if own_team_color == TeamColor.RED else Proto.Team.RED,
            RobotAttributes.TEAM_UNKNOWN: Proto.Team.UNKNOWN_TEAM,
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
