#!/usr/bin/env python3
#  -*- coding: utf8 -*-
from math import pi

import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == "__main__":
    rospy.init_node('motor_position_sender')
    ns = rospy.get_param("ns")
    joint_goal_publisher = rospy.Publisher(ns +'/controller/command', JointTrajectory, queue_size=10, tcp_nodelay=True)

    i = -1.5
    while not rospy.is_shutdown():
        msg = JointTrajectoryPoint()
        msg.positions = [i, -0.4]
        msg.time_from_start.nsecs = 10000000
        traj_msg = JointTrajectory()
        # make an array with String objects (ros message type)
        traj_msg.joint_names = ['HeadPan', 'HeadTilt']
        traj_msg.points = []
        traj_msg.points.append(msg)
        traj_msg.header.stamp = rospy.Time.now()

        rospy.logwarn(traj_msg)

        joint_goal_publisher.publish(traj_msg)
        rospy.sleep(0.2)
        if i < 1.5:
            i += 0.1
        else:
            i = -1.5

    rospy.spin()