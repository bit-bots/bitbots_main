from humanoid_league_team_communication.converter.message_to_team_data_converter import MessageToTeamDataConverter
from humanoid_league_team_communication.converter.state_to_message_converter import StateToMessageConverter
from humanoid_league_team_communication.robocup_extension_pb2 import Action, OffensiveSide, Role, Team

from humanoid_league_msgs.msg import ObstacleRelative, Strategy


class RobocupProtocolConverter:

    def __init__(self):
        self.team_mapping = ((Team.UNKNOWN_TEAM, ObstacleRelative.ROBOT_UNDEFINED),
                             (Team.BLUE, ObstacleRelative.ROBOT_CYAN), (Team.RED, ObstacleRelative.ROBOT_MAGENTA))
        self.role_mapping = (
            (Role.ROLE_UNDEFINED, Strategy.ROLE_UNDEFINED),
            (Role.ROLE_IDLING, Strategy.ROLE_IDLING),
            (Role.ROLE_OTHER, Strategy.ROLE_OTHER),
            (Role.ROLE_STRIKER, Strategy.ROLE_STRIKER),
            (Role.ROLE_SUPPORTER, Strategy.ROLE_SUPPORTER),
            (Role.ROLE_DEFENDER, Strategy.ROLE_DEFENDER),
            (Role.ROLE_GOALIE, Strategy.ROLE_GOALIE),
        )
        self.action_mapping = (
            (Action.ACTION_UNDEFINED, Strategy.ACTION_UNDEFINED),
            (Action.ACTION_POSITIONING, Strategy.ACTION_POSITIONING),
            (Action.ACTION_GOING_TO_BALL, Strategy.ACTION_GOING_TO_BALL),
            (Action.ACTION_TRYING_TO_SCORE, Strategy.ACTION_TRYING_TO_SCORE),
            (Action.ACTION_WAITING, Strategy.ACTION_WAITING),
            (Action.ACTION_KICKING, Strategy.ACTION_KICKING),
            (Action.ACTION_SEARCHING, Strategy.ACTION_SEARCHING),
            (Action.ACTION_LOCALIZING, Strategy.ACTION_LOCALIZING),
        )
        self.side_mapping = (
            (OffensiveSide.SIDE_UNDEFINED, Strategy.SIDE_UNDEFINED),
            (OffensiveSide.SIDE_LEFT, Strategy.SIDE_LEFT),
            (OffensiveSide.SIDE_MIDDLE, Strategy.SIDE_MIDDLE),
            (OffensiveSide.SIDE_RIGHT, Strategy.SIDE_RIGHT),
        )

        mappings = {
            "team_mapping": dict(self.team_mapping),
            "role_mapping": dict(self.role_mapping),
            "action_mapping": dict(self.action_mapping),
            "side_mapping": dict(self.side_mapping),
        }

        reverse_mapping = lambda mapping: dict((b, a) for a, b in mapping)
        reverse_mappings = {
            "team_mapping": reverse_mapping(self.team_mapping),
            "role_mapping": reverse_mapping(self.role_mapping),
            "action_mapping": reverse_mapping(self.action_mapping),
            "side_mapping": reverse_mapping(self.side_mapping),
        }

        self.convert_from_message = MessageToTeamDataConverter(**mappings).convert
        self.convert_to_message = StateToMessageConverter(**reverse_mappings).convert
