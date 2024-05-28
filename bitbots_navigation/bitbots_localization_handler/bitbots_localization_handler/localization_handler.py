#!/usr/bin/env python3
import os

import rclpy
from dynamic_stack_decider.dsd import DSD
from game_controller_hl_interfaces.msg import GameState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu

from bitbots_localization_handler import localization_dsd
from bitbots_localization_handler.localization_dsd.localization_blackboard import LocalizationBlackboard
from bitbots_msgs.msg import RobotControlState


def init(node: Node):
    # Create blackboard to store state
    blackboard = LocalizationBlackboard(node)

    # Create DSD
    dsd = DSD(blackboard, "debug/dsd/localization", node)

    # Register dsd related files
    dsd.register_actions(localization_dsd.actions.__path__[0])
    dsd.register_decisions(localization_dsd.decisions.__path__[0])
    dsd.load_behavior(os.path.join(localization_dsd.__path__[0], "localization.dsd"))

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
    node.create_subscription(
        Imu, "/imu/data", blackboard._callback_imu, 1, callback_group=MutuallyExclusiveCallbackGroup()
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
