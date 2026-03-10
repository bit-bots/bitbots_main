import os

import rclpy
from ament_index_python import get_package_share_directory
from nodes.walk_node import WalkNode


def main():
    rclpy.init()

    walk_policy_path = os.path.join(get_package_share_directory("bitbots_rl_walk"), "models", "wolfgang_walk_ppo.onnx")
    # kick_policy_path = os.path.join(get_package_share_directory("bitbots_rl_walk"), "models", "wolfgang_kick_ppo.onnx")

    walk_node = WalkNode(walk_policy_path)
    # kick_node = RLNode(kick_policy_path)

    rclpy.spin(walk_node)
    walk_node.destroy()

    rclpy.try_shutdown()
