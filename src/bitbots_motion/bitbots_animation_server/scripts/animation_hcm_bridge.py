#!/usr/bin/env python3

"""
This script subscribes to the topic "animation" and publishes the received joint commands to the motor command topic, skipping the HCM.
"""

import rclpy
from rclpy.node import Node

# List of all joint names. Do not change the order as it is important for Gazebo
from bitbots_msgs.msg import Animation, JointCommand

JOINT_NAMES = [
    "head_yaw_joint",
    "head_pitch_joint",
    "l_shoulder_pitch_joint",
    "l_shoulder_roll_joint",
    "l_elbow_joint",
    "r_shoulder_pitch_joint",
    "r_shoulder_roll_joint",
    "r_elbow_joint",
    "l_hip_roll_joint",
    "l_hip_roll_joint",
    "l_hip_pitch_joint",
    "l_calf_joint",
    "l_ankle_pitch_joint",
    "l_ankle_roll_joint",
    "r_hip_roll_joint",
    "r_hip_roll_joint",
    "r_hip_pitch_joint",
    "r_calf_joint",
    "r_ankle_pitch_joint",
    "r_ankle_roll_joint",
]


class AnimationHcmBridge(Node):
    def __init__(self):
        rclpy.init(args=None)
        super().__init__("animation")
        self.joint_publisher = self.create_publisher(JointCommand, "joint_command", 1)
        self.joint_command_msg = JointCommand()
        self.joint_command_msg.joint_names = JOINT_NAMES
        self.joint_command_msg.positions = [0.0] * len(JOINT_NAMES)
        self.joint_command_msg.velocities = [-1.0] * len(JOINT_NAMES)
        self.joint_command_msg.accelerations = [-1.0] * len(JOINT_NAMES)
        self.joint_command_msg.max_torques = [-1.0] * len(JOINT_NAMES)

        self.create_subscription(Animation, "animation", self.animation_cb, 10)

    def animation_cb(self, msg: Animation):
        self.joint_command_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(msg.position.joint_names)):
            name = msg.position.joint_names[i]
            self.joint_command_msg.positions[JOINT_NAMES.index(name)] = msg.position.points[0].positions[i]
            self.joint_command_msg.velocities[JOINT_NAMES.index(name)] = -1

        self.joint_publisher.publish(self.joint_command_msg)


if __name__ == "__main__":
    bridge = AnimationHcmBridge()
    rclpy.spin(bridge)
