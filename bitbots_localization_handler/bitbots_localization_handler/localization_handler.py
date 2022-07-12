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
from bitbots_utils.utils import get_parameters_from_other_node
from rclpy.parameter import Parameter

def init(node: Node):
    node.get_logger().info("Starting localization handler")

    global_params_dict = get_parameters_from_other_node(node, "parameter_blackboard", ["field_length", "team_id"])
    node.get_logger().warn(f"{global_params_dict}")
    global_params = [("field_length", global_params_dict["field_length"]),
                     ("team_id", global_params_dict["team_id"])]
    node.declare_parameters(parameters=global_params, namespace="")

    blackboard = LocalizationBlackboard(node)

    dirname = get_package_share_directory("bitbots_localization_handler")
    node.get_logger().info(dirname)
    dsd = DSD(blackboard, "debug/dsd/localization", node)
    dsd.register_actions(os.path.join(dirname, 'actions'))
    dsd.register_decisions(os.path.join(dirname, 'decisions'))
    dsd.load_behavior(os.path.join(dirname, 'localization.dsd'))

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
