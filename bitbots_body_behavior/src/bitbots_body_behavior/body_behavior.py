#!/usr/bin/env python

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
from humanoid_league_msgs.msg import BallRelative, GameState, HeadMode, Strategy, TeamData,\
    PlayAnimationAction, GoalPartsRelative, RobotControlState
from move_base_msgs.msg import MoveBaseActionFeedback
from actionlib_msgs.msg import GoalID

from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider import dsd


if __name__ == "__main__":
    rospy.init_node("Bodybehavior")
    D = dsd.DSD(BodyBlackboard(), '/debug/dsd/body_behavior')

    D.blackboard.team_data.strategy_sender = rospy.Publisher("strategy", Strategy, queue_size=2)
    D.blackboard.blackboard.head_pub = rospy.Publisher("head_mode", HeadMode, queue_size=10)
    D.blackboard.pathfinding.pathfinding_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    D.blackboard.pathfinding.pathfinding_cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)

    dirname = os.path.dirname(os.path.realpath(__file__))

    D.register_actions(os.path.join(dirname, "actions"))
    D.register_decisions(os.path.join(dirname, "decisions"))

    D.load_behavior(os.path.join(dirname, "main.dsd"))


    # TODO: callbacks away from the blackboard!
    rospy.Subscriber("ball_relative", BallRelative, D.blackboard.world_model.ball_callback)
    rospy.Subscriber("goal_parts_relative", GoalPartsRelative, D.blackboard.world_model.goal_parts_callback)
    rospy.Subscriber("gamestate", GameState, D.blackboard.gamestate.gamestate_callback)
    rospy.Subscriber("team_data", TeamData, D.blackboard.team_data.team_data_callback)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, D.blackboard.world_model.position_callback)
    rospy.Subscriber("robot_state", RobotControlState, D.blackboard.blackboard.robot_state_callback)
    rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback, D.blackboard.pathfinding.feedback_callback)

    D.blackboard.animation.server = actionlib.SimpleActionClient("bitbots_animation", PlayAnimationAction)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        D.update()
        rate.sleep()
