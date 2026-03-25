import os

import rclpy
from ament_index_python import get_package_share_directory
from nodes.walk_node import WalkNode


def main():
    rclpy.init()

    wolfgang_config = os.path.join(get_package_share_directory("bitbots_rl_motion"), "config", "wolfgang_config.yaml")
    # kick_policy_path = os.path.join(get_package_share_directory("bitbots_rl_motion"), "models", "wolfgang_kick_ppo.onnx")

    walk_node = WalkNode(wolfgang_config)
    # kick_node = RLNode(kick_policy_path)

    rclpy.spin(walk_node)
    walk_node.destroy()

    rclpy.try_shutdown()
