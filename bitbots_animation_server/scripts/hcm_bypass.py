#!/usr/bin/env python3

import rospy
from bitbots_msgs.msg import JointCommand
import math
from humanoid_league_msgs.msg import Animation as AnimationMsg

rospy.init_node("hcm_bypass")

pub = rospy.Publisher("/DynamixelController/command", JointCommand, queue_size=1)

def cb(msg):
    msg2 = JointCommand()
    msg2.joint_names = msg.position.joint_names
    msg2.positions = msg.position.points[0].positions
    msg2.velocities = [-1] * len(msg2.joint_names)
    msg2.accelerations = [-1] * len(msg2.joint_names)
    msg2.max_currents = [-1] * len(msg2.joint_names)
    pub.publish(msg2)


#rospy.Subscriber("/animation_motor_goals", Animation, cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("animation", AnimationMsg, cb, queue_size=1, tcp_nodelay=True)
rospy.spin()
