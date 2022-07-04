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
from bitbots_localization.localization_dsd.localization_blackboard import LocalizationBlackboard


class LocalizationHandler(Node):
    def __init__(self, node):
        super(LocalizationHandler).__init__(self, automatically_declare_parameters_from_overrides=True)

        self.get_logger().info("Starting localization handler")

        self.blackboard = LocalizationBlackboard(node)

        dirname = get_package_share_directory("bitbots_localization") + "/localization_dsd"
        self.get_logger().info(dirname)
        self.dsd = DSD(self.blackboard, "debug/dsd/localization", self)
        self.dsd.register_actions(os.path.join(dirname, 'actions'))
        self.dsd.register_decisions(os.path.join(dirname, 'decisions'))
        self.dsd.load_behavior(os.path.join(dirname, 'localization.dsd'))


        self.create_subscription("pose_with_covariance", PoseWithCovarianceStamped, self._callback_pose)
        self.create_subscription("gamestate", GameState, self.blackboard.gamestate.gamestate_callback)
        self.create_subscription("robot_state", RobotControlState, self._callback_robot_control_state)

        self.create_timer(1/25.0, self.dsd.update)

    def _callback_pose(self, msg):
        self.blackboard.last_pose_update_time = msg.header.stamp
        self.blackboard.poseX = msg.pose.pose.position.x
        self.blackboard.poseY = msg.pose.pose.position.y
        self.blackboard.orientation = msg.pose.pose.orientation
        self.blackboard.covariance = msg.pose.covariance

    def _callback_robot_control_state(self, msg):
        self.blackboard.robot_control_state = msg.state


def main(args=None):
    rclpy.init(args=None)
    # needed to init rclcpp ros for moveit_bindings
    node = LocalizationHandler("bitbots_localization_handler", automatically_declare_parameters_from_overrides=True)
    multi_executor = MultiThreadedExecutor()
    multi_executor.add_node(node)

    try:
        multi_executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()