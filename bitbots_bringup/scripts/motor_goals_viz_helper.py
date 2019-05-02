#!/usr/bin/env python3

import argparse

import rospy
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import JointCommand
from humanoid_league_msgs.msg import Animation

JOINT_NAMES = ['LHipYaw', 'LHipRoll', 'LHipPitch', 'LKnee', 'LAnklePitch', 'LAnkleRoll', 'RHipYaw', 'RHipRoll', 'RHipPitch', 'RKnee', 'RAnklePitch', 'RAnkleRoll', 'LShoulderRoll', 'LShoulderPitch', 'LElbow', 'RShoulderRoll', 'RShoulderPitch', 'RElbow', 'HeadPan', 'HeadTilt']

class MotorVizHelper:
    def __init__(self):
        # get rid of addional ROS args when used in launch file
        args0 = rospy.myargv()

        parser = argparse.ArgumentParser()
        parser.add_argument("--walking", "-w", help="Directly get walking motor goals", action="store_true")
        parser.add_argument("--animation", "-a", help="Directly get animation motor goals", action="store_true")
        parser.add_argument("--head", "-k", help="Directly get head motor goals", action="store_true")
        args = parser.parse_args(args0[1:])

        rospy.init_node("motor_viz_helper", anonymous=False)
        self.joint_publisher = rospy.Publisher('joint_states', JointState, queue_size=1, tcp_nodelay=True)
        if args.walking:
            rospy.Subscriber("walking_motor_goals", JointCommand, self.joint_command_cb, queue_size=1, tcp_nodelay=True)
        if args.animation:
            rospy.Subscriber("animation_motor_goals", JointCommand, self.animation_cb, queue_size=1, tcp_nodelay=True)
        if args.head:
            rospy.Subscriber("head_motor_goals", JointCommand, self.animation_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/DynamixelController/command", JointCommand, self.joint_command_cb, queue_size=1, tcp_nodelay=True)

        self.joint_state_msg = JointState()
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_msg.name = JOINT_NAMES
        self.joint_state_msg.position = [0] * 20
        self.joint_publisher.publish(self.joint_state_msg)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.joint_state_msg.header.stamp = rospy.Time.now()
            self.joint_publisher.publish(self.joint_state_msg)
            rate.sleep()

    def joint_command_cb(self, msg: JointCommand):
        self.joint_state_msg.header = msg.header
        self.joint_state_msg.name = JOINT_NAMES
        for i in range(len(msg.joint_names)):
            name = msg.joint_names[i]
            self.joint_state_msg.position[JOINT_NAMES.index(name)] = msg.positions[i]

    def animation_cb(self, msg: Animation):
        self.joint_state_msg.header = msg.header
        self.joint_state_msg.name = msg.position.joint_names
        self.joint_state_msg.position = msg.position.points[0].positions

helper = MotorVizHelper()

