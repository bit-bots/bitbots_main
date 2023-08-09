from typing import Optional

from bitbots_blackboard.capsules.animation_capsule import AnimationCapsule
from bitbots_blackboard.capsules.misc_capsule import MiscCapsule
from bitbots_blackboard.capsules.game_status_capsule import GameStatusCapsule
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
        # References
        self.node = node
        self.tf_buffer = tf_buffer

        # Config
        self.config = get_parameter_dict(node, "body")
        self.base_footprint_frame: str = self.node.get_parameter("base_footprint_frame").value
        self.map_frame: str = self.node.get_parameter("map_frame").value
        self.odom_frame: str = self.node.get_parameter("odom_frame").value
        self.in_sim: bool = self.node.get_parameter("use_sim_time").value

        # Capsules
        self.misc = MiscCapsule(node)
        self.gamestate = GameStatusCapsule(node)
        self.animation = AnimationCapsule(node)
        self.kick = KickCapsule(self)
        self.world_model = WorldModelCapsule(self)
        self.costmap = CostmapCapsule(self)
        self.pathfinding = PathfindingCapsule(self, node)
        self.team_data = TeamDataCapsule(node)
