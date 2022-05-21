import rclpy
from bitbots_utils.utils import get_parameter_dict
from rclpy.node import Node
from rclpy.publisher import Publisher
from bio_ik_msgs.srv import GetIK
from bitbots_blackboard.capsules.animation_capsule import AnimationCapsule
from bitbots_blackboard.capsules.blackboard_capsule import BlackboardCapsule
from bitbots_blackboard.capsules.game_status_capsule import GameStatusCapsule
from bitbots_blackboard.capsules.head_capsule import HeadCapsule
from bitbots_blackboard.capsules.kick_capsule import KickCapsule
from bitbots_blackboard.capsules.pathfinding_capsule import PathfindingCapsule
from bitbots_blackboard.capsules.team_data_capsule import TeamDataCapsule
from bitbots_blackboard.capsules.world_model_capsule import WorldModelCapsule

from rclpy.action import ActionClient
from humanoid_league_msgs.action import PlayAnimation


class BodyBlackboard:
    def __init__(self, node: Node):
        self.node = node

        self.config = self.node.get_parameter("behavior/body").get_parameter_value().double_value
        self.base_footprint_frame = self.node.get_parameter("~base_footprint_frame").get_parameter_value().double_value
        self.map_frame = self.node.get_parameter("~map_frame").get_parameter_value().double_value
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.animation = AnimationCapsule()
        self.kick = KickCapsule(self)
        self.world_model = WorldModelCapsule(self)
        self.pathfinding = PathfindingCapsule(self)
        self.world_model = WorldModelCapsule(self)
        self.team_data = TeamDataCapsule()
        # animations
        self.animation_action_client = ActionClient(self, PlayAnimation, 'animation')
        self.goalie_arms_animation = self.node.get_parameter(
            "Animations/Goalie/goalieArms").get_parameter_value().double_value
        self.goalie_falling_right_animation = self.node.get_parameter(
            "Animations/Goalie/fallRight").get_parameter_value().double_value
        self.goalie_falling_left_animation = self.node.get_parameter(
            "Animations/Goalie/fallLeft").get_parameter_value().double_value
        self.goalie_falling_center_animation = self.node.get_parameter(
            "Animations/Goalie/fallCenter").get_parameter_value().double_value
        self.cheering_animation = self.node.get_parameter("Animations/Misc/cheering").get_parameter_value().double_value
        self.init_animation = self.node.get_parameter("Animations/Misc/init").get_parameter_value().double_value

        self.dynup_action_client = None
        self.dynup_cancel_pub = None  # type: Publisher
        self.hcm_deactivate_pub = None  # type: Publisher


class HeadBlackboard:
    def __init__(self, node: Node):
        self.node = node
        self.config = get_parameter_dict(node, "head")
        self.head_capsule = HeadCapsule(self)
        self.world_model = WorldModelCapsule(self)