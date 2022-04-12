#!/usr/bin/env python3
#  -*- coding: utf8 -*-
import rclpy
from rclpy.node import Node
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == "__main__":
    rclpy.init(args=None)
    joint_goal_publisher = self.create_publisher(JointTrajectory, '/minibot/joint_trajectory_action_controller/command', 10)

    msg = JointTrajectoryPoint()
    msg.positions = [90]
    traj_msg = JointTrajectory()
    # make an array with String objects (ros message type)
    traj_msg.joint_names = ['HeadTilt']
    traj_msg.points = []
    traj_msg.points.append(msg)
    traj_msg.header.stamp = self.get_clock().now()

    joint_goal_publisher.publish(traj_msg)
    self.get_logger().warn(traj_msg)
    joint_goal_publisher.publish(traj_msg)
    joint_goal_publisher.publish(traj_msg)
    joint_goal_publisher.publish(traj_msg)



    self.get_clock().sleep_for(Duration(seconds=1)