#!/usr/bin/env python3
import os
import rclpy
import tf2_ros as tf2
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python import get_package_share_directory

from humanoid_league_msgs.msg import GameState, RobotControlState
from geometry_msgs.msg import PoseWithCovarianceStamped

from dynamic_stack_decider.dsd import DSD
from bitbots_localization_handler.localization_dsd.localization_blackboard import LocalizationBlackboard


def init(node):
    node.get_logger().info("Starting localization handler")

    blackboard = LocalizationBlackboard(node)

    dirname = get_package_share_directory("bitbots_localization") + "/localization_dsd"
    node.get_logger().info(dirname)
    dsd = DSD(blackboard, "debug/dsd/localization", node)
    dsd.register_actions(os.path.join(dirname, 'actions'))
    dsd.register_decisions(os.path.join(dirname, 'decisions'))
    dsd.load_behavior(os.path.join(dirname, 'localization.dsd'))

    node.create_subscription("pose_with_covariance", PoseWithCovarianceStamped, blackboard._callback_pose)
    node.create_subscription("gamestate", GameState, blackboard.gamestate.gamestate_callback)
    node.create_subscription("robot_state", RobotControlState, blackboard._callback_robot_control_state)

    return dsd

def main(args=None):
    rclpy.init(args=None)
    # needed to init rclcpp ros for moveit_bindings
    node = Node("bitbots_localization_handler", automatically_declare_parameters_from_overrides=True)
    multi_executor = MultiThreadedExecutor()
    multi_executor.add_node(node)

    dsd = init(node)
    node.create_timer(1 / 25.0, dsd.update)

    try:
        multi_executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
