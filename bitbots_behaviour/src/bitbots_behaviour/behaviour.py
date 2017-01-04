#!/usr/bin/env python3.6
"""
BehaviourModule
^^^^^^^^^^^^^^^
.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Startet das Verhalten
"""
from abstract.stack_machine_module import StackMachineModule
from body.decisions.common.duty_decider import DutyDecider
import rospy
from geometry_msgs.msg import Twist, Pose2D
from humanoid_league_msgs.msg import BallRelative, ObstacleRelative, GameState, Speak, Role
from nav_msgs.msg import Odometry


class BehaviourModule(StackMachineModule):
    def __init__(self):
        self.set_start_module(DutyDecider)
        super(BehaviourModule, self).__init__()

        #rospy.Subscriber("/odometry", Odometry, self.connector.walking.walking_callback) # todo vermutlich unnötig
        rospy.Subscriber("/ball_relative", BallRelative, self.connector.vision.ball_callback)
        rospy.Subscriber("/obstacle_relative", ObstacleRelative, self.connector.vision.obstacle_callback)
        rospy.Subscriber("/Gamestate", GameState, self.connector.gamestate.gamestate_callback)

        self.connector.speaker = rospy.Publisher("speak", Speak)
        self.connector.team_data.role_sender = rospy.Publisher("/role", Role)
        self.connector.walking.pub_walking_objective = rospy.Publisher("/navigation_goal", Pose2D)
        self.connector.walking.pub_walkin_params = rospy.Publisher("/cmd_vel", Twist)

        rospy.init_node("Behaviour")

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == "__main__":
    bm = BehaviourModule()
    bm.run()
