#!/usr/bin/env python3

import argparse

import rospy
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import JointCommand
from humanoid_league_msgs.msg import Animation

class MotorVizHelper:
    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--w", help="Directly get walking motor goals", action="store_true")
        parser.add_argument("--a", help="Directly get animation motor goals", action="store_true")
        args = parser.parse_args()

        rospy.init_node("motor_viz_helper", anonymous=False)
        self.joint_state_msg = JointState()
        self.joint_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)
        if args.w:
            rospy.Subscriber("walking_motor_goals", JointCommand, self.joint_command_cb, queue_size=1)
        elif args.a:
            rospy.Subscriber("animation_motor_goals", JointCommand, self.animation_cb, queue_size=1)
        else:
            rospy.Subscriber("/DynamixelController/command", JointCommand, self.joint_command_cb, queue_size=1)

        rospy.spin()

    def joint_command_cb(self, msg:JointCommand):
        self.joint_state_msg.header = msg.header
        self.joint_state_msg.name = msg.joint_names
        self.joint_state_msg.position = msg.positions
        self.joint_publisher.publish(self.joint_state_msg)

    def animation_cb(self, msg:JointCommand):
        self.joint_state_msg.header = msg.header
        self.joint_state_msg.name = msg.position.joint_names
        self.joint_state_msg.position = msg.position.points[0].positions
        self.joint_publisher.publish(self.joint_state_msg)

helper = MotorVizHelper()

