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
    update_time = rospy.Time.now()
    joint_command = JointCommand()
    joint_command.joint_names = joint_list
    joint_command.positions = [0] * 20
    joint_command.velocities = [-1] * 20

    def update_gazebo_command(command):
        # type: (JointCommand) -> Float64MultiArray
        global update_time
        msg = Float64MultiArray()
        for name, position, velocity in zip(command.joint_names, command.positions, command.velocities):
            if velocity == -1:
                positions[joint_list.index(name)] = position
            else:
                old_pos = positions[joint_list.index(name)]
                time_delta = (rospy.Time.now() - update_time).to_sec()
                max_rad = time_delta * velocity
                if position - old_pos > max_rad:
                    new_pos = old_pos + max_rad
                elif position - old_pos < -max_rad:
                    new_pos = old_pos - max_rad
                else:
                    new_pos = position
                positions[joint_list.index(name)] = new_pos
        update_time = rospy.Time.now()
        msg.data = positions
        return msg

    def joint_command_cb(msg):
        # type: (JointCommand) -> None
        for i in range(len(msg.joint_names)):
            name = msg.joint_names[i]
            joint_command.positions[joint_list.index(name)] = msg.positions[i]
            joint_command.velocities[joint_list.index(name)] = msg.velocities[i]


    goal_subscriber = rospy.Subscriber("DynamixelController/command", JointCommand, joint_command_cb, tcp_nodelay=True)
    goal_publisher = rospy.Publisher('JointGroupController/command', Float64MultiArray, queue_size=10, tcp_nodelay=True)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            # catch exeption of moving backwarts in time, when restarting simulator
            msg = update_gazebo_command(joint_command)
            goal_publisher.publish(msg)
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn(
                "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
        except rospy.exceptions.ROSInterruptException:
            exit()
