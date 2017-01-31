#!/usr/bin/env python
#  -*- coding: utf8 -*-
from math import pi

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == "__main__":
    rospy.init_node('motor_position_sender')
    joint_goal_publisher = rospy.Publisher('/motion_motor_goals', JointTrajectory, queue_size=10)

    i = 0
    while not rospy.is_shutdown():
        msg = JointTrajectoryPoint()
        msg.positions = [i]
        traj_msg = JointTrajectory()
        # make an array with String objects (ros message type)
        traj_msg.joint_names = ['LElbow']
        traj_msg.points = []
        traj_msg.points.append(msg)
        traj_msg.header.stamp = rospy.Time.now()

        rospy.logwarn(traj_msg)

        joint_goal_publisher.publish(traj_msg)
        rospy.sleep(0.2)
        if i < pi:
            i += 0.1
        else:
            i = 0

    rospy.spin()