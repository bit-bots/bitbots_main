from typing import Optional

from bitbots_blackboard.capsules.animation_capsule import AnimationCapsule
from bitbots_blackboard.capsules.blackboard_capsule import BlackboardCapsule
from bitbots_blackboard.capsules.game_status_capsule import GameStatusCapsule
from bitbots_blackboard.capsules.head_capsule import HeadCapsule
from bitbots_blackboard.capsules.kick_capsule import KickCapsule
from bitbots_blackboard.capsules.pathfinding_capsule import PathfindingCapsule
from bitbots_blackboard.capsules.team_data_capsule import TeamDataCapsule
from bitbots_blackboard.capsules.world_model_capsule import WorldModelCapsule
from bitbots_blackboard.capsules.costmap_capsule import CostmapCapsule
from bitbots_utils.utils import get_parameter_dict
from rclpy.action import ActionClient
from rclpy.node import Node
import tf2_ros as tf2
from rclpy.publisher import Publisher


class BodyBlackboard:
    def __init__(self, node: Node, tf_buffer: tf2.Buffer):
        self.node = node
        self.tf_buffer = tf_buffer

        self.config = get_parameter_dict(node, "body")
        self.base_footprint_frame: str = self.node.get_parameter("base_footprint_frame").value
        self.map_frame: str = self.node.get_parameter("map_frame").value
        self.blackboard = BlackboardCapsule(node)
        self.gamestate = GameStatusCapsule(node)
        self.animation = AnimationCapsule(node)
        self.kick = KickCapsule(self)
        self.world_model = WorldModelCapsule(self)
        self.costmap = CostmapCapsule(self)
        self.pathfinding = PathfindingCapsule(self, node)
        self.team_data = TeamDataCapsule(node)
        self.goalie_arms_animation: str = self.node.get_parameter("Animations.Goalie.goalieArms").value
        self.goalie_falling_right_animation: str = self.node.get_parameter("Animations.Goalie.fallRight").value
        self.goalie_falling_left_animation: str = self.node.get_parameter("Animations.Goalie.fallLeft").value
        self.goalie_falling_center_animation: str = self.node.get_parameter("Animations.Goalie.fallCenter").value
        self.cheering_animation: str = self.node.get_parameter("Animations.Misc.cheering").value
        self.init_animation: str = self.node.get_parameter("Animations.Misc.init").value

        self.dynup_action_client: Optional[ActionClient] = None
        self.dynup_cancel_pub: Optional[Publisher] = None
        self.hcm_deactivate_pub: Optional[Publisher] = None
        
        self.lookat_action_client: Optional[ActionClient] = None
        


class HeadBlackboard:
    def __init__(self, node: Node, tf_buffer: tf2.Buffer):
        self.node = node
        self.tf_buffer = tf_buffer
        self.config = get_parameter_dict(node, "head")
        self.head_capsule = HeadCapsule(self)
        self.world_model = WorldModelCapsule(self)
        self.costmap = CostmapCapsule(self)
