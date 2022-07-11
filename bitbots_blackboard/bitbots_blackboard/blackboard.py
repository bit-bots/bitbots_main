import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.publisher import Publisher

from bitbots_utils.utils import get_parameter_dict
from humanoid_league_msgs.action import PlayAnimation
from bitbots_msgs.action import Dynup

from bitbots_blackboard.capsules.animation_capsule import AnimationCapsule
from bitbots_blackboard.capsules.blackboard_capsule import BlackboardCapsule
from bitbots_blackboard.capsules.game_status_capsule import GameStatusCapsule
from bitbots_blackboard.capsules.head_capsule import HeadCapsule
from bitbots_blackboard.capsules.kick_capsule import KickCapsule
from bitbots_blackboard.capsules.pathfinding_capsule import PathfindingCapsule
from bitbots_blackboard.capsules.team_data_capsule import TeamDataCapsule
from bitbots_blackboard.capsules.world_model_capsule import WorldModelCapsule

class BodyBlackboard:
    def __init__(self, node: Node):
        self.node = node

        self.config = get_parameter_dict(node, "body")
        self.base_footprint_frame = self.node.get_parameter("base_footprint_frame").get_parameter_value().string_value
        self.map_frame = self.node.get_parameter("map_frame").get_parameter_value().string_value
        self.blackboard = BlackboardCapsule(node)
        self.gamestate = GameStatusCapsule(node)
        self.animation = AnimationCapsule(node)
        self.kick = KickCapsule(self)
        self.world_model = WorldModelCapsule(self)
        self.pathfinding = PathfindingCapsule(self, node)
        self.team_data = TeamDataCapsule(node)
        # animations
        self.animation_action_client = ActionClient(node, PlayAnimation, 'animation')
        self.dynup_action_client = ActionClient(node, Dynup, 'dynup')
        self.goalie_arms_animation = self.node.get_parameter(
            "Animations.Goalie.goalieArms").get_parameter_value().string_value
        self.goalie_falling_right_animation = self.node.get_parameter(
            "Animations.Goalie.fallRight").get_parameter_value().string_value
        self.goalie_falling_left_animation = self.node.get_parameter(
            "Animations.Goalie.fallLeft").get_parameter_value().string_value
        self.goalie_falling_center_animation = self.node.get_parameter(
            "Animations.Goalie.fallCenter").get_parameter_value().string_value
        self.cheering_animation = self.node.get_parameter("Animations.Misc.cheering").get_parameter_value().string_value
        self.init_animation = self.node.get_parameter("Animations.Misc.init").get_parameter_value().string_value

        self.dynup_action_client = None
        self.dynup_cancel_pub = None  # type: Publisher
        self.hcm_deactivate_pub = None  # type: Publisher


class HeadBlackboard:
    def __init__(self, node: Node):
        self.node = node
        self.config = get_parameter_dict(node, "head")
        self.head_capsule = HeadCapsule(self)
        self.world_model = WorldModelCapsule(self)
