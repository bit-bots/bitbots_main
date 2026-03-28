import os

import rclpy
from rclpy.executors import MultiThreadedExecutor
from ament_index_python import get_package_share_directory
from nodes.kick_node import KickNode
from nodes.walk_node import WalkNode


def main():
    rclpy.init()

    wolfgang_config = os.path.join(get_package_share_directory("bitbots_rl_motion"), "configs", "wolfgang_config.yaml")

    #walk_node = WalkNode(wolfgang_config)
    kick_node = KickNode(wolfgang_config)

    executor = MultiThreadedExecutor()
    #executor.add_node(walk_node)
    executor.add_node(kick_node)

    try:
        executor.spin()
    finally:
        #walk_node.destroy_node()
        kick_node.destroy_node()
        rclpy.shutdown()
