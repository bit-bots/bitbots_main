#!/usr/bin/env python3

"""
BehaviorModule
^^^^^^^^^^^^^^
.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Starts the body behavior
"""

import os

import tf2_ros as tf2
import rclpy
from actionlib_msgs.msg import GoalID
from ament_index_python import get_package_share_directory
from bitbots_blackboard.blackboard import BodyBlackboard
from geometry_msgs.msg import (PoseWithCovarianceStamped, Twist,
                               TwistWithCovarianceStamped)
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from soccer_vision_3d_msgs.msg import RobotArray
from std_msgs.msg import Bool, Empty, Float32
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import Marker
from bitbots_tf_listener import TransformListener

from bitbots_msgs.action import Dynup, LookAt
from dynamic_stack_decider.dsd import DSD
from humanoid_league_msgs.msg import (GameState, HeadMode, RobotControlState,
                                      Strategy, TeamData)


class BodyDSD:
    def __init__(self, node:Node):
        self.counter = 0
        self.step_running = False
        self.node = node

        self.tf_buffer = tf2.Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, node)

        blackboard = BodyBlackboard(node, self.tf_buffer)
        self.dsd = DSD(blackboard, 'debug/dsd/body_behavior', node) #TODO: use config

        self.dsd.blackboard.team_data.strategy_sender = node.create_publisher(Strategy, "strategy", 2)
        self.dsd.blackboard.team_data.time_to_ball_publisher = node.create_publisher(Float32, "time_to_ball", 2)
        self.dsd.blackboard.blackboard.head_pub = node.create_publisher(HeadMode, "head_mode", 10)
        self.dsd.blackboard.pathfinding.direct_cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 1)
        self.dsd.blackboard.pathfinding.pathfinding_pub = node.create_publisher(PoseStamped, 'goal_pose', 1)
        self.dsd.blackboard.pathfinding.pathfinding_cancel_pub = node.create_publisher(Empty, 'pathfinding/cancel', 1)
        self.dsd.blackboard.pathfinding.ball_obstacle_active_pub = node.create_publisher(Bool, "ball_obstacle_active", 1)
        self.dsd.blackboard.pathfinding.approach_marker_pub = node.create_publisher(Marker, "debug/approach_point", 10)
        self.dsd.blackboard.dynup_cancel_pub = node.create_publisher(GoalID, 'dynup/cancel', 1)
        self.dsd.blackboard.hcm_deactivate_pub = node.create_publisher(Bool, 'hcm_deactivate', 1)

        dirname = get_package_share_directory("bitbots_body_behavior")

        self.dsd.register_actions(os.path.join(dirname, "actions"))
        self.dsd.register_decisions(os.path.join(dirname, "decisions"))

        self.dsd.load_behavior(os.path.join(dirname, "main.dsd"))
        self.dsd.blackboard.dynup_action_client = ActionClient(node, Dynup, 'dynup')
        
        self.dsd.blackboard.lookat_action_client = ActionClient(node, LookAt, 'look_at_goal')

        node.create_subscription(
            PoseWithCovarianceStamped,
            "ball_position_relative_filtered",
            blackboard.world_model.ball_filtered_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup())
        node.create_subscription(
            GameState,
            "gamestate",
            blackboard.gamestate.gamestate_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup())
        node.create_subscription(
            TeamData,
            "team_data",
            blackboard.team_data.team_data_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup())
        node.create_subscription(
            PoseWithCovarianceStamped,
            "pose_with_covariance",
            blackboard.world_model.pose_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup())
        node.create_subscription(
            RobotArray,
            "robots_relative_filtered",
            blackboard.costmap.robot_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup())
        node.create_subscription(
            RobotControlState,
            "robot_state",
            blackboard.blackboard.robot_state_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup())
        node.create_subscription(TwistWithCovarianceStamped,
            node.get_parameter("body.ball_movement_subscribe_topic").get_parameter_value().string_value,
            blackboard.world_model.ball_twist_callback,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup())
        node.create_subscription(
            Twist,
            "cmd_vel",
            blackboard.pathfinding.cmd_vel_cb,
            qos_profile=1,
            callback_group=MutuallyExclusiveCallbackGroup())

    def loop(self):
        try:
            self.dsd.update()
            self.dsd.blackboard.team_data.publish_strategy()
            self.dsd.blackboard.team_data.publish_time_to_ball()
            self.counter = (self.counter + 1) % self.dsd.blackboard.config['time_to_ball_divider']
            if self.counter == 0:
                self.dsd.blackboard.pathfinding.calculate_time_to_ball()
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.node.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=None)
    node = Node("body_behavior", automatically_declare_parameters_from_overrides=True)
    body_dsd = BodyDSD(node)
    node.create_timer(1/60.0, body_dsd.loop, callback_group=MutuallyExclusiveCallbackGroup(), clock=node.get_clock())
    # Number of executor threads is the number of MutiallyExclusiveCallbackGroups + 2 threads needed by the tf listener and executor
    multi_executor = MultiThreadedExecutor(num_threads=12)
    multi_executor.add_node(node)

    try:
        multi_executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()

