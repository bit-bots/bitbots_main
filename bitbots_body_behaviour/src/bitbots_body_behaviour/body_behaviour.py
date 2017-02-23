#!/usr/bin/env python3
"""
BehaviourModule
^^^^^^^^^^^^^^^
.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Starts the body behaviour
"""
import rospy
# from bitbots_animation.animation_node import PlayAnimationAction
from bitbots_body_behaviour.body.decisions.common.duty_decider import DutyDecider
from geometry_msgs.msg import Twist, Pose2D
from humanoid_league_msgs.msg import BallRelative, ObstacleRelative, GameState, Speak, Role, HeadMode
from bitbots_stackmachine.stack_machine_module import StackMachineModule
from bitbots_common.connector.connector import BodyConnector


class BehaviourModule(StackMachineModule):
    def __init__(self):
        super().__init__()
        self.connector = BodyConnector()
        self.connector.config = rospy.get_param("Behaviour")

        self.connector.speaker = rospy.Publisher("speak", Speak, queue_size=3)
        self.connector.team_data.role_sender = rospy.Publisher("/role", Role, queue_size=2)
        self.connector.walking.pub_walking_objective = rospy.Publisher("/navigation_goal", Pose2D, queue_size=3)
        self.connector.walking.pub_walkin_params = rospy.Publisher("/cmd_vel", Twist, queue_size=6)
        self.connector.head_pub = rospy.Publisher("/head_duty", HeadMode, queue_size=10)

        self.set_start_module(DutyDecider)

        rospy.Subscriber("/ball_relative", BallRelative, self.connector.vision.ball_callback)
        rospy.Subscriber("/obstacle_relative", ObstacleRelative, self.connector.vision.obstacle_callback)
        rospy.Subscriber("/Gamestate", GameState, self.connector.gamestate.gamestate_callback)

        # self.connector.animation.server = actionlib.SimpleActionClient("bitbots_animation", PlayAnimationAction)

        rospy.init_node("Bodybehaviour")

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == "__main__":
    bm = BehaviourModule()
    bm.run()
