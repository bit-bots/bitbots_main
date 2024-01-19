#!/usr/bin/env python3
import os

import rclpy
from ament_index_python import get_package_share_directory
from dynamic_stack_decider.dsd import DSD
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from bitbots_localization_handler.localization_dsd.localization_blackboard import LocalizationBlackboard
from bitbots_msgs.msg import GameState, RobotControlState


def init(node: Node):
    # Create blackboard to store state
    blackboard = LocalizationBlackboard(node)

    # Create DSD
    dsd = DSD(blackboard, "debug/dsd/localization", node)
    # Get the location of the package to load the dsd files
    dirname = os.path.join(get_package_share_directory("bitbots_localization_handler"), "localization_dsd")
    # Register dsd related files
    dsd.register_actions(os.path.join(dirname, "actions"))
    dsd.register_decisions(os.path.join(dirname, "decisions"))
    dsd.load_behavior(os.path.join(dirname, "localization.dsd"))

    # Create subscribers
    node.create_subscription(
        PoseWithCovarianceStamped,
        "pose_with_covariance",
        blackboard._callback_pose,
        1,
        callback_group=ReentrantCallbackGroup(),
    )
    node.create_subscription(
        GameState, "gamestate", blackboard.gamestate.gamestate_callback, 1, callback_group=ReentrantCallbackGroup()
    )
    node.create_subscription(
        RobotControlState,
        "robot_state",
        blackboard._callback_robot_control_state,
        1,
        callback_group=ReentrantCallbackGroup(),
    )

    return dsd


def main(args=None):
    rclpy.init(args=None)

    # Create node
    node = Node("bitbots_localization_handler", automatically_declare_parameters_from_overrides=True)

    # Create DSD, blackboard, etc.
    dsd = init(node)

    # Create timer to update DSD
    node.create_timer(1 / 25.0, dsd.update, callback_group=MutuallyExclusiveCallbackGroup())

    # Create executor
    multi_executor = MultiThreadedExecutor(4)
    multi_executor.add_node(node)

    # Spin the executor
    try:
        multi_executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
