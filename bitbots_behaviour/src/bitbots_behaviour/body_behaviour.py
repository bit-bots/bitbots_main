#!/usr/bin/env python3
"""
BehaviourModule
^^^^^^^^^^^^^^^
.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Startet das Verhalten
"""
import actionlib
import rospy
from bitbots_animation.animation_node import PlayAnimationAction
from body.decisions.common.duty_decider import DutyDecider
from geometry_msgs.msg import Twist, Pose2D
from humanoid_league_msgs.msg import BallRelative, ObstacleRelative, GameState, Speak, Role
from stackmachine.stack_machine_module import StackMachineModule


class BehaviourModule(StackMachineModule):
    def __init__(self):
        self.set_start_module(DutyDecider)
        super(BehaviourModule, self).__init__()

        rospy.Subscriber("/ball_relative", BallRelative, self.connector.vision.ball_callback)
        rospy.Subscriber("/obstacle_relative", ObstacleRelative, self.connector.vision.obstacle_callback)
        rospy.Subscriber("/Gamestate", GameState, self.connector.gamestate.gamestate_callback)

        self.connector.speaker = rospy.Publisher("speak", Speak)
        self.connector.team_data.role_sender = rospy.Publisher("/role", Role)
        self.connector.walking.pub_walking_objective = rospy.Publisher("/navigation_goal", Pose2D)
        self.connector.walking.pub_walkin_params = rospy.Publisher("/cmd_vel", Twist)

        self.connector.animation.server = actionlib.SimpleActionClient("bitbots_animation", PlayAnimationAction)

        rospy.init_node("Behaviour")

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == "__main__":
    bm = BehaviourModule()
    bm.run()
