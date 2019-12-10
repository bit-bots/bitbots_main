#!/usr/bin/env python3
#  -*- coding: utf8 -*-
import rospy
from bitbots_msgs.msg import JointCommand
from sensor_msgs.msg import JointState


def goal_cb(msg):
    state_msg = JointState()
    state_msg.name = msg.joint_names
    state_msg.position = msg.positions
    state_msg.header = msg.header
    state_msg.header.stamp = rospy.Time.now()

    state_publisher.publish(state_msg)


if __name__ == "__main__":
    rospy.init_node('goals_to_joint_states')
    state_publisher = rospy.Publisher("/joint_states", JointState, queue_size=1)
    sub = rospy.Subscriber("/DynamixelController/command", JointCommand, callback=goal_cb, queue_size=1,
                           tcp_nodelay=True)
    rospy.spin()
