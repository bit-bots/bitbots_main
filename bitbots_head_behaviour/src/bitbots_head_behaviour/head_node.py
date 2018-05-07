#!/usr/bin/env python2.7
# -*- coding:utf-8 -*-
import rospy
from bitbots_common.connector.connector import HeadConnector
from bitbots_head_behaviour.decisions.head_duty_decider import HeadDutyDecider
from bitbots_stackmachine.stack_machine_module import StackMachineModule
from humanoid_league_msgs.msg import HeadMode, BallRelative, ObstacleRelative, BallInImage, BallsInImage, GoalRelative
from sensor_msgs.msg import JointState, CameraInfo
from trajectory_msgs.msg import JointTrajectory


class HeadNode(StackMachineModule):
    def __init__(self):
        super(HeadNode, self).__init__()
        self.connector = HeadConnector()
        self.connector.config = rospy.get_param("Behaviour")

        self.connector.head.position_publisher = rospy.Publisher("head_motor_goals", JointTrajectory, queue_size=10)

        rospy.init_node("Headbehaviour")

        rospy.Subscriber("joint_states", JointState, self.connector.head.joint_state_cb)
        rospy.Subscriber("head_duty", HeadMode, self.connector.head.cb_headmode, queue_size=10)
        rospy.Subscriber("ball_relative", BallRelative, self.connector.personal_model.ball_callback)
        rospy.Subscriber("ball_relative", BallRelative, self.connector.world_model.ball_relative_cb)
        rospy.Subscriber("goal_relative", GoalRelative, self.connector.personal_model.goal_callback)
        rospy.Subscriber("obstacle_relative", ObstacleRelative, self.connector.personal_model.obstacle_callback)

        self.set_start_module(HeadDutyDecider)

    def run(self):
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            rospy.logdebug("######Run Headbehaviour")
            self.update()
            rate.sleep()

if __name__ == "__main__":
    hb = HeadNode()
    hb.run()

