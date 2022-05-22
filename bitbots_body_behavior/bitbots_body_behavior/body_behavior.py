#!/usr/bin/env python3

"""
BehaviorModule
^^^^^^^^^^^^^^
.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Starts the body behavior
"""

import os

import rclpy
from rclpy.node import Node

from actionlib_msgs.msg import GoalID
from bitbots_msgs.action import Dynup 
from tf2_geometry_msgs import PoseStamped
from humanoid_league_msgs.msg import GameState, HeadMode, Strategy, TeamData,\
    RobotControlState, PoseWithCertaintyArray
#from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider import dsd
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

def main(args=None):
    rclpy.init(args=None)
    node = Node("body_behavior", automatically_declare_parameters_from_overrides=True)

    blackboard = BodyBlackboard(node)
    D = dsd.DSD(blackboard, 'debug/dsd/body_behavior')

    D.blackboard.team_data.strategy_sender = node.create_publisher(Strategy, "strategy", 2)
    D.blackboard.team_data.time_to_ball_publisher = node.create_publisher(Float32, "time_to_ball", 2)
    D.blackboard.blackboard.head_pub = node.create_publisher(HeadMode, "head_mode", 10)
    D.blackboard.pathfinding.direct_cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 1)
    D.blackboard.pathfinding.pathfinding_pub = node.create_publisher(PoseStamped, 'move_base_simple/goal', 1)
    D.blackboard.pathfinding.pathfinding_cancel_pub = node.create_publisher(GoalID, 'move_base/cancel', 1)
    D.blackboard.pathfinding.ball_obstacle_active_pub = node.create_publisher(Bool, "ball_obstacle_active", 1)
    D.blackboard.pathfinding.keep_out_area_pub = node.create_publisher(PointCloud2, "keep_out_area", 1)
    D.blackboard.pathfinding.approach_marker_pub = node.create_publisher(Marker, "debug/approach_point", 10)
    D.blackboard.dynup_cancel_pub = node.create_publisher(GoalID, 'dynup/cancel', 1)
    D.blackboard.hcm_deactivate_pub = node.create_publisher(Bool, 'hcm_deactivate', 1)

    dirname = os.path.dirname(os.path.realpath(__file__))

    D.register_actions(os.path.join(dirname, "actions"))
    D.register_decisions(os.path.join(dirname, "decisions"))

    D.load_behavior(os.path.join(dirname, "main.dsd"))

    D.blackboard.dynup_action_client = actionlib.SimpleActionClient('dynup', DynUpAction)

    # TODO: callbacks away from the blackboard!
    node.create_subscription(PoseWithCovarianceStamped, "ball_position_relative_filtered", D.blackboard.world_model.ball_filtered_callback)
    node.create_subscription(PoseWithCertaintyArray, "goal_posts_relative", D.blackboard.world_model.goalposts_callback)
    node.create_subscription(GameState, "gamestate", D.blackboard.gamestate.gamestate_callback)
    node.create_subscription(TeamData, "team_data", D.blackboard.team_data.team_data_callback)
    node.create_subscription(PoseWithCovarianceStamped, "pose_with_covariance", D.blackboard.world_model.pose_callback)
    node.create_subscription(PointCloud2, "robot_obstacles", D.blackboard.world_model.robot_obstacle_callback)
    node.create_subscription(RobotControlState, "robot_state", D.blackboard.blackboard.robot_state_callback)
    node.create_subscription(TwistWithCovarianceStamped,
        node.get_parameter("body.ball_movement_subscribe_topic").get_parameter_value().string_value,
        D.blackboard.world_model.ball_twist_callback)
    node.create_subscription(MoveBaseActionFeedback, "move_base/feedback", D.blackboard.pathfinding.feedback_callback)
    node.create_subscription(MoveBaseActionResult, "move_base/result", D.blackboard.pathfinding.status_callback)
    node.create_subscription(Twist, "cmd_vel", D.blackboard.pathfinding.cmd_vel_cb)

    rate = node.create_rate(125)
    counter = 0
    while rclpy.ok():
        D.update()
        D.blackboard.team_data.publish_strategy()
        D.blackboard.team_data.publish_time_to_ball()
        counter = (counter + 1) % D.blackboard.config['time_to_ball_divider']
        if counter == 0:
            D.blackboard.pathfinding.calculate_time_to_ball()
        rate.sleep()
