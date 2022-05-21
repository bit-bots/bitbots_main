#!/usr/bin/env python3

"""
BehaviorModule
^^^^^^^^^^^^^^
.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Starts the body behavior
"""

import actionlib
import os
import rospy
from bitbots_msgs.msg import DynUpAction
from tf2_geometry_msgs import PoseStamped
from humanoid_league_msgs.msg import GameState, HeadMode, Strategy, TeamData,\
    RobotControlState, PoseWithCertainty, PoseWithCertaintyArray
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider import dsd
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path

if __name__ == "__main__":
    rospy.init_node("Bodybehavior")
    D = dsd.DSD(BodyBlackboard(), 'debug/dsd/body_behavior')

    D.blackboard.team_data.strategy_sender = rospy.Publisher("strategy", Strategy, queue_size=2)
    D.blackboard.team_data.time_to_ball_publisher = rospy.Publisher("time_to_ball", Float32, queue_size=2)
    D.blackboard.blackboard.head_pub = rospy.Publisher("head_mode", HeadMode, queue_size=10)
    D.blackboard.pathfinding.direct_cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    D.blackboard.pathfinding.pathfinding_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    D.blackboard.pathfinding.pathfinding_cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
    D.blackboard.pathfinding.ball_obstacle_active_pub = rospy.Publisher("ball_obstacle_active", Bool, queue_size=1)
    D.blackboard.pathfinding.keep_out_area_pub = rospy.Publisher("keep_out_area", PointCloud2, queue_size=1)
    D.blackboard.pathfinding.approach_marker_pub = rospy.Publisher("debug/approach_point", Marker, queue_size=10)

    D.blackboard.dynup_cancel_pub = rospy.Publisher('dynup/cancel', GoalID, queue_size=1)
    D.blackboard.hcm_deactivate_pub = rospy.Publisher('hcm_deactivate', Bool, queue_size=1)

    dirname = os.path.dirname(os.path.realpath(__file__))

    D.register_actions(os.path.join(dirname, "actions"))
    D.register_decisions(os.path.join(dirname, "decisions"))

    D.load_behavior(os.path.join(dirname, "main.dsd"))

    D.blackboard.dynup_action_client = actionlib.SimpleActionClient('dynup', DynUpAction)

    # TODO: callbacks away from the blackboard!
    rospy.Subscriber("ball_position_relative_filtered", PoseWithCovarianceStamped, D.blackboard.world_model.ball_filtered_callback)
    rospy.Subscriber("goal_posts_relative", PoseWithCertaintyArray, D.blackboard.world_model.goalposts_callback)
    rospy.Subscriber("gamestate", GameState, D.blackboard.gamestate.gamestate_callback)
    rospy.Subscriber("team_data", TeamData, D.blackboard.team_data.team_data_callback)
    rospy.Subscriber("pose_with_covariance", PoseWithCovarianceStamped, D.blackboard.world_model.pose_callback)
    rospy.Subscriber("robot_obstacles", PointCloud2, D.blackboard.world_model.robot_obstacle_callback)
    rospy.Subscriber("robot_state", RobotControlState, D.blackboard.blackboard.robot_state_callback)
    rospy.Subscriber(
        rospy.get_param("behavior/body/ball_movement_subscribe_topic"),
        TwistWithCovarianceStamped,
        D.blackboard.world_model.ball_twist_callback)
    rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback, D.blackboard.pathfinding.feedback_callback)
    rospy.Subscriber("move_base/result", MoveBaseActionResult, D.blackboard.pathfinding.status_callback)
    rospy.Subscriber("cmd_vel", Twist, D.blackboard.pathfinding.cmd_vel_cb)

    rate = rospy.Rate(125)
    counter = 0
    path_to_ball_service_response = None
    while not rospy.is_shutdown():
        D.update()
        D.blackboard.team_data.publish_strategy()
        D.blackboard.team_data.publish_time_to_ball()
        counter = (counter + 1) % D.blackboard.config['time_to_ball_divider']
        if counter == 0:
            D.blackboard.pathfinding.calculate_time_to_ball()
        rate.sleep()
