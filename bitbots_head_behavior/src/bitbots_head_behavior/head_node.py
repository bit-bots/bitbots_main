#!/usr/bin/env python3
"""
This is the ROS-Node which contains the head behavior, starts the appropriate DSD, initializes the HeadBlackboard
and subscribes to head_behavior specific ROS-Topics.
"""
import os

import rclpy
from rclpy.node import Node

from bitbots_blackboard.blackboard import HeadBlackboard
from dynamic_stack_decider.dsd import DSD

from humanoid_league_msgs.msg import HeadMode as HeadModeMsg, PoseWithCertainty, PoseWithCertaintyArray
from bitbots_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init, roscpp_shutdown


def run(dsd):
    """
    Main run-loop

    :returns: Never
    """
    node = Node("head_node")
    rate = node.create_rate(60)
    while rclpy.ok():
        dsd.update()
        rate.sleep()
    # Also stop cpp node
    roscpp_shutdown()


def init():
    """
    Initialize new components needed for head_behavior:
    blackboard, dsd, rostopic subscriber
    """
    rclpy.init(args=None)
    node = Node("head_node2")
    # This is a general purpose initialization function provided by moved
    # It is used to correctly initialize roscpp which is used in the collision checker module
    roscpp_init('collision_checker', [])
    blackboard = HeadBlackboard()

    rclpy.Subscriber('head_mode', HeadModeMsg, blackboard.head_capsule.head_mode_callback, queue_size=1)
    rclpy.Subscriber("ball_position_relative_filtered", PoseWithCovarianceStamped, blackboard.world_model.ball_filtered_callback)
    rclpy.Subscriber('joint_states', JointState, blackboard.head_capsule.joint_state_callback)
    blackboard.head_capsule.position_publisher = self.create_publisher(JointCommand, "head_motor_goals", 10)
    blackboard.head_capsule.visual_compass_record_trigger = self.create_publisher(Header, blackboard.config['visual_compass_trigger_topic'], 5)

    dirname = os.path.dirname(os.path.realpath(__file__))

    dsd = DSD(blackboard, 'debug/dsd/head_behavior')
    dsd.register_actions(os.path.join(dirname, 'actions'))
    dsd.register_decisions(os.path.join(dirname, 'decisions'))
    dsd.load_behavior(os.path.join(dirname, 'head_behavior.dsd'))

    node.get_logger().debug("Head Behavior completely loaded")
    return dsd


if __name__ == '__main__':
    run(init())
