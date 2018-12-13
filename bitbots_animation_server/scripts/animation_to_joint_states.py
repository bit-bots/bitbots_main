#!/usr/bin/env python
#  -*- coding: utf8 -*-
import rospy
from humanoid_league_msgs.msg import Animation
from sensor_msgs.msg import JointState

def anim_cb(msg):    
    state_msg = JointState()
    state_msg.name = msg.position.joint_names
    state_msg.position = msg.position.points[0].positions
    state_msg.header = msg.header

    state_publisher.publish(state_msg)

if __name__ == "__main__":
    rospy.init_node('motor_position_sender')    
    state_publisher = rospy.Publisher("/joint_states", JointState, queue_size=1)    
    sub = rospy.Subscriber("/animation", Animation, callback=anim_cb, queue_size=1, tcp_nodelay=True)
    rospy.spin()
    