#!/usr/bin/env python
#  -*- coding: utf8 -*-
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == "__main__":
    rospy.init_node('sender')
    joint_goal_publisher = rospy.Publisher('/minibot/joint_trajectory_action_controller/command', JointTrajectory, queue_size=10)

    msg = JointTrajectoryPoint()
    msg.positions = [90]
    traj_msg = JointTrajectory()
    # make an array with String objects (ros message type)
    traj_msg.joint_names = ['HeadTilt']
    traj_msg.points = []
    traj_msg.points.append(msg)
    traj_msg.header.stamp = rospy.Time.now()

    joint_goal_publisher.publish(traj_msg)
    rospy.logwarn(traj_msg)
    joint_goal_publisher.publish(traj_msg)
    joint_goal_publisher.publish(traj_msg)
    joint_goal_publisher.publish(traj_msg)



    rospy.sleep(2)