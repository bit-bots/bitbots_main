#!/usr/bin/env python3
#  -*- coding: utf8 -*-
from math import pi

import rclpy
from rclpy.node import Node
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == "__main__":
    rclpy.init(args=None)
    ns = rospy.get_param("ns")
    joint_goal_publisher = self.create_publisher(JointTrajectory, ns +'/controller/command', 10, tcp_nodelay=True)

    i = -1.5
    while rclpy.ok():
        msg = JointTrajectoryPoint()
        msg.positions = [i, -0.4]
        msg.time_from_start.nsecs = 10000000
        traj_msg = JointTrajectory()
        # make an array with String objects (ros message type)
        traj_msg.joint_names = ['HeadPan', 'HeadTilt']
        traj_msg.points = []
        traj_msg.points.append(msg)
        traj_msg.header.stamp = self.get_clock().now()

        rospy.logwarn(traj_msg)

        joint_goal_publisher.publish(traj_msg)
        rospy.sleep(0.2)
        if i < 1.5:
            i += 0.1
        else:
            i = -1.5

    rclpy.spin(self)