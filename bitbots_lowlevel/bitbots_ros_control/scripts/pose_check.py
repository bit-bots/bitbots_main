#!/usr/bin/env python3

"""
Moves the robots to a bunch of different poses to check if the robot is configured/build/calibrated as expected
"""


import math

import rclpy
import rclpy.publisher
from colorama import Fore, Style
from rclpy.node import Node
from rclpy.publisher import Publisher

from bitbots_msgs.msg import JointCommand

SPEED = 1.0
DYNAMIXEL_CMD_TOPIC = "/DynamixelController/command"
JOINT_NAMES = [
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


def move_to_joint_position(publisher: Publisher, joint_goals: dict[str, float | int]):
    assert set(joint_goals.keys()) == set(JOINT_NAMES), "Joint goals do not match the robots joints"
    publisher.publish(
        JointCommand(
            joint_names=JOINT_NAMES,
            velocities=[SPEED] * len(JOINT_NAMES),
            accelerations=[-1] * len(JOINT_NAMES),
            max_currents=[-1] * len(JOINT_NAMES),
            positions=[float(joint_goals[joint]) for joint in JOINT_NAMES],
        )
    )


def main():
    rclpy.init(args=None)
    node = Node("pose_check")
    pub = node.create_publisher(JointCommand, DYNAMIXEL_CMD_TOPIC, 1)

    try:
        input(
            f"{Fore.BLUE}We now start with the calibration check!\n\n"
            f"{Fore.GREEN}The robot will first move into a t-pose.\n"
            f"{Fore.RED}Make sure that the surroundings are clear and the robot is able to move freely. "
            f"Also be cautions and be prepared to hold the robot in case it is not stable.\n\n{Style.RESET_ALL}"
            f"{Style.BRIGHT}Press enter to continue."
        )
        # Arms
        print(
            f"\n{Fore.YELLOW}The robot will now move into a t-pose. 🥵{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
        )
        # TODO knee init pose offset for straight legs
        # Move to T-Pose
        move_to_joint_position(
            pub,
            {
                "HeadPan": 0,
                "HeadTilt": 0,
                "LAnklePitch": 0,
                "LAnkleRoll": 0,
                "LElbow": -math.pi / 2,
                "LHipPitch": 0,
                "LHipRoll": 0,
                "LHipYaw": 0,
                "LKnee": 0,
                "LShoulderPitch": 0,
                "LShoulderRoll": -math.pi / 2,
                "RAnklePitch": 0,
                "RAnkleRoll": 0,
                "RElbow": math.pi / 2,
                "RHipPitch": 0,
                "RHipRoll": 0,
                "RHipYaw": 0,
                "RKnee": 0,
                "RShoulderPitch": 0,
                "RShoulderRoll": math.pi / 2,
            },
        )

        input(f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}")

        print(
            f"\n{Fore.YELLOW}The robot will now move into a arms up pose. 🥵{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
        )

        input(f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}")

        # arms up
        move_to_joint_position(
            pub,
            {
                "HeadPan": 0,
                "HeadTilt": 0,
                "LAnklePitch": 0,
                "LAnkleRoll": 0,
                "LElbow": -math.pi / 2,
                "LHipPitch": 0,
                "LHipRoll": 0,
                "LHipYaw": 0,
                "LKnee": 0,
                "LShoulderPitch": 0,
                "LShoulderRoll": -math.pi,
                "RAnklePitch": 0,
                "RAnkleRoll": 0,
                "RElbow": math.pi / 2,
                "RHipPitch": 0,
                "RHipRoll": 0,
                "RHipYaw": 0,
                "RKnee": 0,
                "RShoulderPitch": 0,
                "RShoulderRoll": math.pi,
            },
        )

        input(f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}")

        # TODO arms to the front
        # TODO arms down
        # TODO elbows 90 degree

        # head
        # TODO head right
        # TODO head left
        # TODO head up
        # TODO head down

        # legs
        # TODO print caution to now lift robot up
        # TODO knees 90 degrees
        # TODO men split legs
        # TODO women split legs

    except KeyboardInterrupt:
        print(f"\n\n{Fore.RED}Closing...")
    finally:
        print(Style.RESET_ALL)


if __name__ == "__main__":
    main()
