#!/usr/bin/env python2.7
# -*- coding:utf-8 -*-
"""
BehaviourModule
^^^^^^^^^^^^^^^
.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Starts the body behaviour
"""
import sys

import bitbots_body_behaviour
import rospy
from bitbots_stackmachine.stack_machine import StackMachine
from bitbots_body_behaviour.decisions.common.duty_decider import DutyDecider
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose2D
from humanoid_league_msgs.msg import BallRelative, GameState, Speak, HeadMode, Strategy, TeamData
from bitbots_connector.connector import BodyConnector


class BehaviourModule(StackMachine):
    def __init__(self):
        super(BehaviourModule, self).__init__(BodyConnector(), debug_topic="/debug_body_behaviour")

        self.connector.speaker.speaker = rospy.Publisher("speak", Speak, queue_size=3)
        self.connector.team_data.strategy_sender = rospy.Publisher("strategy", Strategy, queue_size=2)
        self.connector.blackboard.head_pub = rospy.Publisher("head_duty", HeadMode, queue_size=10)
        self.connector.pathfinding.pathfinding_simple_pub = rospy.Publisher('bitbots_pathfinding/relative_goal', Pose2D, queue_size=10)

        if len(sys.argv) > 1:
            duty = sys.argv[1]
            rospy.loginfo("Forcing %s behaviour" % duty)
        else:
            duty = None

        bitbots_body_behaviour.decisions.common.duty_decider.duty = duty

        self.set_start_element(DutyDecider)
        rospy.init_node("Bodybehaviour")

        rospy.Subscriber("ball_relative", BallRelative, self.connector.world_model.ball_callback)
        rospy.Subscriber("gamestate", GameState, self.connector.gamestate.gamestate_callback)
        rospy.Subscriber("team_data", TeamData, self.connector.team_data.team_data_callback)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.connector.world_model.position_callback)

        # self.connector.animation.server = actionlib.SimpleActionClient("bitbots_animation", PlayAnimationAction)


    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == "__main__":
    bm = BehaviourModule()
    bm.run()
