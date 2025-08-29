#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from bitbots_msgs.msg import JointCommand

DYNAMIXEL_CMD_TOPIC = "/DynamixelController/command"


class PredefinedCommands:
    __ids__ = [
        "HeadPan",
        "HeadTilt",
        "LShoulderPitch",
        "LShoulderRoll",
        "LElbow",
        "RShoulderPitch",
        "RShoulderRoll",
        "RElbow",
        "LHipYaw",
        "LHipRoll",
        "LHipPitch",
        "LKnee",
        "LAnklePitch",
        "LAnkleRoll",
        "RHipYaw",
        "RHipRoll",
        "RHipPitch",
        "RKnee",
        "RAnklePitch",
        "RAnkleRoll",
    ]
    __velocity__ = 5.0
    __accelerations__ = -1.0
    __max_currents__ = -1.0

    Zero = JointCommand(
        joint_names=__ids__,
        velocities=[__velocity__] * len(__ids__),
        accelerations=[__accelerations__] * len(__ids__),
        max_currents=[__max_currents__] * len(__ids__),
        positions=[0.0] * len(__ids__),
    )

    Walkready = JointCommand(
        joint_names=__ids__,
        velocities=[__velocity__] * len(__ids__),
        accelerations=[__accelerations__] * len(__ids__),
        max_currents=[__max_currents__] * len(__ids__),
        positions=[
            0.0,  # HeadPan
            0.0,  # HeadTilt
            0.0,  # LShoulderPitch
            0.0,  # LShoulderRoll
            0.79,  # LElbow
            0.0,  # RShoulderPitch
            0.0,  # RShoulderRoll
            -0.79,  # RElbow
            -0.0112,  # LHipYaw
            0.0615,  # LHipRoll
            0.4732,  # LHipPitch
            1.0058,  # LKnee
            -0.4512,  # LAnklePitch
            0.0625,  # LAnkleRoll
            0.0112,  # RHipYaw
            -0.0615,  # RHipRoll
            -0.4732,  # RHipPitch
            -1.0059,  # RKnee
            0.4512,  # RAnklePitch
            -0.0625,  # RAnkleRoll
        ],
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Send a bitbots_msgs/JointCommand to our ros-control node in a loop")
    parser.add_argument(
        "--once", action="store_true", default=False, help="Only send the message once instead of in a loop"
    )
    parser.add_argument(
        "-c",
        "--command",
        type=str,
        help="Command to send. Use one of the available choices",
        choices=[c for c in PredefinedCommands.__dict__ if not c.startswith("__")],
        default="Zero",
    )

    return parser.parse_args()


def main():
    args = parse_args()
    joint_command = PredefinedCommands.__dict__[args.command]

    rclpy.init(args=None)
    node = Node("send_joint_command")
    pub = node.create_publisher(JointCommand, DYNAMIXEL_CMD_TOPIC, 1)

    while pub.get_subscription_count() < 1:
        node.get_logger().info(f"Waiting until subscribers connect to {DYNAMIXEL_CMD_TOPIC}", once=True)
        node.get_clock().sleep_for(Duration(seconds=0.5))
    # just to make sure
    node.get_clock().sleep_for(Duration(seconds=1))

    node.get_logger().info(f"Sending controller commands of type {args.command} now.")
    print(joint_command)

    while rclpy.ok():
        joint_command.header.stamp = node.get_clock().now().to_msg()
        pub.publish(joint_command)
        node.get_clock().sleep_for(Duration(seconds=0.5))

        if args.once:
            return


if __name__ == "__main__":
    main()
