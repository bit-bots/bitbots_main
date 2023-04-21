#!/usr/bin/env python3
"""
This is the ROS-Node which contains the head behavior, starts the appropriate DSD, initializes the HeadBlackboard
and subscribes to head_behavior specific ROS-Topics.
"""
import os

import tf2_ros as tf2
from ament_index_python import get_package_share_directory
from bitbots_blackboard.blackboard import HeadBlackboard
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import rclpy
from bitbots_msgs.msg import JointCommand
from dynamic_stack_decider.dsd import DSD
from humanoid_league_msgs.msg import HeadMode as HeadModeMsg


def init(node: Node):
    """
    Initialize new components needed for head_behavior:
    blackboard, dsd, rostopic subscriber
    """
    tf2_buffer = tf2.Buffer(cache_time=Duration(seconds=30))
    tf2_listener = tf2.TransformListener(tf2_buffer, node)

    blackboard = HeadBlackboard(node, tf2_buffer)

    node.create_subscription(
        HeadModeMsg,
        'head_mode',
        blackboard.head_capsule.head_mode_callback,
        1,
        callback_group=MutuallyExclusiveCallbackGroup())
    node.create_subscription(
        PoseWithCovarianceStamped,
        "ball_position_relative_filtered",
        blackboard.world_model.ball_filtered_callback,
        1,
        callback_group=MutuallyExclusiveCallbackGroup())
    node.create_subscription(
        JointState,
        "joint_states",
        blackboard.head_capsule.joint_state_callback,
        1,
        callback_group=MutuallyExclusiveCallbackGroup())
    blackboard.head_capsule.position_publisher = node.create_publisher(
        JointCommand,
        "head_motor_goals",
        10)
    blackboard.head_capsule.visual_compass_record_trigger = node.create_publisher(
        Header, blackboard.config['visual_compass_trigger_topic'], 5)

    dirname = get_package_share_directory("bitbots_head_behavior")

    dsd = DSD(blackboard, 'debug/dsd/head_behavior', node)
    dsd.register_actions(os.path.join(dirname, 'actions'))
    dsd.register_decisions(os.path.join(dirname, 'decisions'))
    dsd.load_behavior(os.path.join(dirname, 'head_behavior.dsd'))

    node.get_logger().debug("Head Behavior completely loaded")
    return dsd


def main(args=None):
    rclpy.init(args=None)
    node = Node("head_node", automatically_declare_parameters_from_overrides=True)
    dsd = init(node)

    node.create_timer(1 / 60.0, dsd.update, callback_group=MutuallyExclusiveCallbackGroup())
    # Number of executor threads is the number of MutiallyExclusiveCallbackGroups + 2 threads the tf listener and executor needs
    multi_executor = MultiThreadedExecutor(num_threads=7)
    multi_executor.add_node(node)


    try:
        multi_executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
