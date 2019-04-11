#!/usr/bin/env python3

import rospy
from bitbots_msgs.msg import JointCommand
import math

rospy.init_node("hcm_bypass")

pub = rospy.Publisher("/DynamixelController/command", JointCommand, queue_size=1)

def cb(msg):
    pub.publish(msg)  


rospy.Subscriber("/walking_motor_goals", JointCommand, cb, queue_size=1, tcp_nodelay=True)
rospy.spin()