#!/usr/bin/env python2.7
# -*- coding:utf-8 -*-
import rospy
from bitbots_connector.connector import HeadConnector
from bitbots_head_behaviour.decisions.head_duty_decider import HeadDutyDecider
from bitbots_stackmachine.stack_machine import StackMachine
from humanoid_league_msgs.msg import HeadMode, BallRelative, ObstacleRelative, GoalRelative
from sensor_msgs.msg import JointState
from bitbots_ros_control.msg import JointCommand


class HeadNode(StackMachine):
    def __init__(self):
        rospy.init_node("Headbehaviour")
        super(HeadNode, self).__init__(HeadConnector(), debug_topic="/debug_head_behaviour")

        self.connector.head.position_publisher = rospy.Publisher("head_motor_goals", JointCommand, queue_size=10)

        rospy.Subscriber("joint_states", JointState, self.connector.head.joint_state_cb)
        rospy.Subscriber("head_duty", HeadMode, self.connector.head.cb_headmode, queue_size=10)
        rospy.Subscriber("ball_relative", BallRelative, self.connector.world_model.ball_callback)
        rospy.Subscriber("goal_relative", GoalRelative, self.connector.world_model.goal_callback)

        self.set_start_element(HeadDutyDecider)

    def run(self):
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            rospy.logdebug("######Run Headbehaviour")
            self.update()
            rate.sleep()


if __name__ == "__main__":
    hb = HeadNode()
    hb.run()

