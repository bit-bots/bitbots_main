#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import JointCommand
from humanoid_league_msgs.msg import Animation


class MotorVizHelper(Node):
    def __init__(self):
        super().__init__('MotorGoalsVizHelper')
        # get rid of additional ROS args when used in launch file

        parser = argparse.ArgumentParser()
        parser.add_argument("--robot-type", "-t", help="What type of robot to use", default="wolfgang")
        parser.add_argument("--walking", "-w", help="Directly get walking motor goals", action="store_true")
        parser.add_argument("--animation", "-a", help="Directly get animation motor goals", action="store_true")
        parser.add_argument("--head", help="Directly get head motor goals", action="store_true")
        parser.add_argument("--kick", help="Directly get kick motor goals", action="store_true")
        parser.add_argument("--dynup", help="Directly get Dynup motor goals", action="store_true")
        parser.add_argument("--all", help="Directly get all motor goals", action="store_true")
        parser.add_argument("--gazebo", help="Publish for Gazebo instead of rviz", action="store_true")
        parser.add_argument("--ros-args", help="just to filter ros args", action="store_true")
        parser.add_argument("-r", help="just to filter ros args", action="store_true")
        parser.add_argument("__node", help="just to filter ros args", action="store_true")
        self.args, unknown = parser.parse_known_args()

        if self.args.robot_type == "wolfgang":
            # List of all joint names. Do not change the order as it is important for Gazebo
            self.joint_names = ['HeadPan', 'HeadTilt', 'LShoulderPitch', 'LShoulderRoll', 'LElbow', 'RShoulderPitch',
                                'RShoulderRoll', 'RElbow', 'LHipYaw', 'LHipRoll', 'LHipPitch', 'LKnee', 'LAnklePitch',
                                'LAnkleRoll', 'RHipYaw', 'RHipRoll', 'RHipPitch', 'RKnee', 'RAnklePitch', 'RAnkleRoll']
            self.joint_goals = [float(0), float(0), float(0), float(0), float(0), float(0), float(0), float(0),
                                float(0), float(0), float(0.7), float(-1), float(-0.4), float(0), float(0), float(0),
                                float(-0.7), float(1), float(0.4), float(0)]
        elif self.args.robot_type == "itandroids":
            self.joint_names = ["rightShoulderPitch[shoulder]", "leftShoulderPitch[shoulder]",
                                "rightShoulderYaw", "leftShoulderYaw", "rightElbowYaw", "leftElbowYaw", "rightHipYaw",
                                "leftHipYaw", "rightHipRoll[hip]", "leftHipRoll[hip]", "rightHipPitch",
                                "leftHipPitch", "rightKneePitch", "leftKneePitch", "rightAnklePitch", "leftAnklePitch",
                                "rightAnkleRoll", "leftAnkleRoll", "neckYaw", "neckPitch"]
            self.joint_goals = [float(0), float(0), float(0), float(0), float(0), float(0), float(0), float(0),
                                float(0), float(0), float(0), float(0), float(0), float(0), float(0), float(0),
                                float(0), float(0), float(0), float(0)]
        else:
            print(f"Unknown robot type {self.args.robot_type}")
            exit()

        if self.args.gazebo:
            self.joint_publisher = self.create_publisher(Float64MultiArray, 'JointGroupController/command', 10)
        else:
            self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

        if self.args.walking or self.args.all:
            self.create_subscription(JointCommand, "walking_motor_goals", self.joint_command_cb, 10)
        if self.args.animation or self.args.all:
            self.create_subscription(Animation, "animation", self.animation_cb, 10)
        if self.args.head or self.args.all:
            self.create_subscription(JointCommand, "head_motor_goals", self.joint_command_cb, 10)
        if self.args.kick or self.args.all:
            self.create_subscription(JointCommand, "kick_motor_goals", self.joint_command_cb, 10)
        if self.args.dynup or self.args.all:
            self.create_subscription(JointCommand, "dynup_motor_goals", self.joint_command_cb, 10)
        self.create_subscription(JointCommand, "/DynamixelController/command", self.joint_command_cb, 10)

        self.joint_state_msg = JointState()
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = self.joint_goals
        if self.args.gazebo:
            self.joint_publisher.publish(self.get_float_array())
        else:
            self.joint_publisher.publish(self.joint_state_msg)

        self.joint_command_msg = JointCommand()
        self.joint_command_msg.joint_names = self.joint_names
        self.joint_command_msg.positions = [float(0)] * 20
        self.joint_command_msg.velocities = [float(-1)] * 20

    def loop(self):
        self.update_time = self.get_clock().now()
        self.update_joint_states(self.joint_command_msg)
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        if self.args.gazebo:
            self.joint_publisher.publish(self.get_float_array())
        else:
            self.joint_publisher.publish(self.joint_state_msg)

    def joint_command_cb(self, msg: JointCommand):
        self.joint_command_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(msg.joint_names)):
            if len(msg.positions) != 0:
                # if msg.positions is 0, torque control is probably used.
                # there, the visualization is not implemented yet.
                name = msg.joint_names[i]
                self.joint_command_msg.positions[self.joint_names.index(name)] = msg.positions[i]
                self.joint_command_msg.velocities[self.joint_names.index(name)] = msg.velocities[i]

    def animation_cb(self, msg: Animation):
        self.joint_command_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(msg.position.joint_names)):
            name = msg.position.joint_names[i]
            self.joint_command_msg.positions[self.joint_names.index(name)] = msg.position.points[0].positions[i]
            self.joint_command_msg.velocities[self.joint_names.index(name)] = -1

    def update_joint_states(self, msg):
        for i in range(len(msg.joint_names)):
            name = msg.joint_names[i]
            if msg.velocities[i] == -1:
                self.joint_state_msg.position[self.joint_names.index(name)] = msg.positions[i]
            else:
                old_pos = self.joint_state_msg.position[self.joint_names.index(name)]
                time_delta = self.get_clock().now() - self.update_time
                time_delta_secs = time_delta.nanoseconds / 1e9
                max_rad = time_delta_secs * msg.velocities[i]
                if msg.positions[i] - old_pos > max_rad:
                    new_pos = old_pos + max_rad
                elif msg.positions[i] - old_pos < -max_rad:
                    new_pos = old_pos - max_rad
                else:
                    new_pos = msg.positions[i]
                self.joint_state_msg.position[self.joint_names.index(name)] = new_pos
        self.update_time = self.get_clock().now()

    def get_float_array(self):
        msg = Float64MultiArray()
        msg.data = self.joint_state_msg.position
        return msg


if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    node = MotorVizHelper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.create_timer(0.01, node.loop)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
