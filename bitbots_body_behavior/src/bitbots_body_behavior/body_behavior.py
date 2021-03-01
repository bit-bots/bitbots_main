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
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_geometry_msgs import PoseStamped
from humanoid_league_msgs.msg import GameState, HeadMode, Strategy, TeamData,\
    RobotControlState, PoseWithCertainty, PoseWithCertaintyArray
from move_base_msgs.msg import MoveBaseActionFeedback
from actionlib_msgs.msg import GoalID

from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider import dsd


if __name__ == "__main__":
    rospy.init_node("Bodybehavior")
    D = dsd.DSD(BodyBlackboard(), 'debug/dsd/body_behavior')

    D.blackboard.team_data.strategy_sender = rospy.Publisher("strategy", Strategy, queue_size=2)
    D.blackboard.blackboard.head_pub = rospy.Publisher("head_mode", HeadMode, queue_size=10)
    D.blackboard.pathfinding.pathfinding_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    D.blackboard.pathfinding.pathfinding_cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)

    dirname = os.path.dirname(os.path.realpath(__file__))

    D.register_actions(os.path.join(dirname, "actions"))
    D.register_decisions(os.path.join(dirname, "decisions"))

    D.load_behavior(os.path.join(dirname, "main.dsd"))


    # TODO: callbacks away from the blackboard!
    rospy.Subscriber("balls_relative", PoseWithCertaintyArray, D.blackboard.world_model.balls_callback)
    rospy.Subscriber("goal_posts_relative", PoseWithCertaintyArray, D.blackboard.world_model.goalposts_callback)
    rospy.Subscriber("gamestate", GameState, D.blackboard.gamestate.gamestate_callback)
    rospy.Subscriber("team_data", TeamData, D.blackboard.team_data.team_data_callback)
    rospy.Subscriber("pose_with_covariance", PoseWithCovarianceStamped, D.blackboard.world_model.pose_callback)
    rospy.Subscriber("robot_state", RobotControlState, D.blackboard.blackboard.robot_state_callback)
    rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback, D.blackboard.pathfinding.feedback_callback)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        D.update()
        rate.sleep()
