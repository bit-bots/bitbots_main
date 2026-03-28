import os

import rclpy
from ament_index_python import get_package_share_directory
from nodes.kick_node import KickNode


def main():
    rclpy.init()

    wolfgang_config = os.path.join(get_package_share_directory("bitbots_rl_motion"), "config", "wolfgang_config.yaml")

    # walk_node = WalkNode(wolfgang_config)
    kick_node = KickNode(wolfgang_config)

    rclpy.spin(kick_node)
    kick_node.destroy()

    rclpy.try_shutdown()
