#!/usr/bin/env python3
import os

import rclpy
from ament_index_python import get_package_share_directory
from bitbots_msgs.msg import GameState, RobotControlState
from dynamic_stack_decider.dsd import DSD
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from bitbots_localization_handler.localization_dsd.localization_blackboard import LocalizationBlackboard


def init(node: Node):
    node.get_logger().info("Starting localization handler")

    blackboard = LocalizationBlackboard(node)

    dirname = get_package_share_directory("bitbots_localization_handler")
    node.get_logger().info(dirname)
    dsd = DSD(blackboard, "debug/dsd/localization", node)
    dsd.register_actions(os.path.join(dirname, "actions"))
    dsd.register_decisions(os.path.join(dirname, "decisions"))
    dsd.load_behavior(os.path.join(dirname, "localization.dsd"))

    node.create_subscription(PoseWithCovarianceStamped, "pose_with_covariance", blackboard._callback_pose, 1)
    node.create_subscription(GameState, "gamestate", blackboard.gamestate.gamestate_callback, 1)
    node.create_subscription(RobotControlState, "robot_state", blackboard._callback_robot_control_state, 1)

    return dsd


def main(args=None):
    rclpy.init(args=None)
    # needed to init rclcpp ros for moveit_bindings
    node = Node("bitbots_localization_handler", automatically_declare_parameters_from_overrides=True)
    dsd = init(node)
    node.create_timer(1 / 25.0, dsd.update)

    multi_executor = MultiThreadedExecutor()
    multi_executor.add_node(node)

    try:
        multi_executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
