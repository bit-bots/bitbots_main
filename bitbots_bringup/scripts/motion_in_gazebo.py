#!/usr/bin/env python
#  -*- coding: utf8 -*-

# This script maps the messages published to DynamixelController/command to /JointGroupController/command.
# This is necessary for the motion of the robot in Gazebo.

import rospy
from std_msgs.msg import Float64MultiArray
from bitbots_msgs.msg import JointCommand



if __name__ == "__main__":    
    rospy.init_node('pub_goals_sim')
    joint_list = ['HeadPan', 'HeadTilt', 'LShoulderPitch', 'LShoulderRoll', 'LElbow', 'RShoulderPitch',
                  'RShoulderRoll', 'RElbow', 'LHipYaw', 'LHipRoll', 'LHipPitch', 'LKnee', 'LAnklePitch',
                  'LAnkleRoll', 'RHipYaw', 'RHipRoll', 'RHipPitch', 'RKnee', 'RAnklePitch', 'RAnkleRoll']

    positions = [0] * 20
    
    def map_data(d_msg):
        msg = Float64MultiArray()    
        d_joint_order = d_msg.joint_names
        i = 0
        # only update newly transmitted positions and retransmit all previos positions
        # Gazebo expects us to always send everything
        for joint in joint_list:
            if joint in d_joint_order:
                positions[i] =d_msg.positions[d_joint_order.index(joint)]
            i += 1

        msg.data = positions
        goal_publisher.publish(msg)

    goal_subscriber = rospy.Subscriber("DynamixelController/command", JointCommand, map_data, tcp_nodelay=True)
    goal_publisher = rospy.Publisher('JointGroupController/command', Float64MultiArray,queue_size=10, tcp_nodelay=True)
    rate = rospy.Rate(20)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        try:
            # catch exeption of moving backwarts in time, when restarting simulator
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn(
                "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
        except rospy.exceptions.ROSInterruptException:
            exit()
