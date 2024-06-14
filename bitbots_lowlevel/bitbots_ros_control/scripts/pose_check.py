#!/usr/bin/env python3

"""
Moves the robots to a bunch of different poses to check if the robot is configured/build/calibrated as expected
"""

import math
import os

import rclpy
import rclpy.publisher
from ament_index_python import get_package_share_directory
from colorama import Back, Fore, Style
from rclpy.node import Node
from rclpy.publisher import Publisher

from bitbots_msgs.msg import HeadMode, JointCommand

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


def num_to_emoji(num: int) -> str:
    emoji_numbers = ["0️⃣", "1️⃣", "2️⃣", "3️⃣", "4️⃣", "5️⃣", "6️⃣", "7️⃣", "8️⃣", "9️⃣"]
    return f'{"".join([emoji_numbers[int(digit)] for digit in str(num)])} '


def move_to_joint_position(publisher: Publisher, joint_goals: dict[str, float | int]):
    assert set(joint_goals.keys()) <= set(JOINT_NAMES), "Joint goals do not match the robots joints"

    publisher.publish(
        JointCommand(
            joint_names=JOINT_NAMES,
            velocities=[SPEED] * len(JOINT_NAMES),
            accelerations=[-1] * len(JOINT_NAMES),
            max_currents=[-1] * len(JOINT_NAMES),
            positions=[float(joint_goals.get(joint, 0)) for joint in JOINT_NAMES],
        )
    )


def main():
    rclpy.init(args=None)
    node = Node("pose_check")
    pub = node.create_publisher(JointCommand, DYNAMIXEL_CMD_TOPIC, 1)

    # Deactivate the head mover
    head_mode_pub = node.create_publisher(HeadMode, "/head_mode", 1)
    head_mode_pub.publish(HeadMode(head_mode=HeadMode.DONT_MOVE))

    step = 1

    # Show bitbot ascii art
    ascii_art = open(os.path.join(get_package_share_directory("bitbots_utils"), "config", "welcome_art.txt")).read()

    print(
        f"\n\n{ascii_art}\n\n"
        f"\n{Fore.LIGHTWHITE_EX}{Style.BRIGHT}{Back.BLUE}                                                {Style.RESET_ALL}\n"
        f"{Fore.LIGHTWHITE_EX}{Style.BRIGHT}{Back.BLUE}    We now start with the calibration check!    {Style.RESET_ALL}\n"
        f"{Fore.LIGHTWHITE_EX}{Style.BRIGHT}{Back.BLUE}    ========================================    {Style.RESET_ALL}\n"
        f"{Fore.LIGHTWHITE_EX}{Style.BRIGHT}{Back.BLUE}                                                {Style.RESET_ALL}\n\n"
        f"{Fore.RED}Make sure that the surroundings are clear and the robot is able to move freely. "
        f"Also be cautions and be prepared to hold the robot in case it is not stable.\n{Style.RESET_ALL}"
    )

    try:
        input(
            f"{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move into a {Style.BRIGHT}t-pose.{Style.RESET_ALL} 🥵\n\n"
            f"{Style.BRIGHT}Press enter to move."
        )
        #################### ARMS ####################

        ##########
        # T-Pose #
        ##########

        # TODO knee init pose offset for straight legs
        move_to_joint_position(
            pub,
            {
                "LElbow": -math.pi / 2,
                "LShoulderRoll": -math.pi / 2,
                "RElbow": math.pi / 2,
                "RShoulderRoll": math.pi / 2,
            },
        )
        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ################
        # Arms-Up-Pose #
        ################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move {Style.BRIGHT}its arms up.{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LElbow": -math.pi / 2,
                "LShoulderRoll": -math.pi,
                "RElbow": math.pi / 2,
                "RShoulderRoll": math.pi,
            },
        )

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ##########################
        # Arms-To-The-Front-Pose #
        ##########################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move {Style.BRIGHT}its arms to the front.{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LElbow": -math.pi / 2,
                "LShoulderPitch": -math.pi / 2,
                "RElbow": math.pi / 2,
                "RShoulderPitch": math.pi / 2,
            },
        )

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        #########################
        # Elbows-90-degree-Pose #
        #########################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move {Style.BRIGHT}its elbows to a 90 degree angle.{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )
        move_to_joint_position(
            pub,
            {
                "LShoulderPitch": -math.pi / 2,
                "RShoulderPitch": math.pi / 2,
            },
        )

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ##################
        # Arms-Down-Pose #
        ##################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move {Style.BRIGHT}its arms down again.{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LElbow": -math.pi / 2,
                "RElbow": math.pi / 2,
            },
        )

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        #################### HEAD ####################

        ###################
        # Head-Right-Pose #
        ###################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move {Style.BRIGHT}its head to the right.{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "HeadPan": -math.pi / 2,
            },
        )

        # TODO Change checks?
        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ##################
        # Head-Left-Pose #
        ##################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move {Style.BRIGHT}its head to the left.{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "HeadPan": math.pi / 2,
            },
        )

        # TODO Change checks?
        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ##################
        # Head-Down-Pose #
        ##################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move {Style.BRIGHT}its head down.{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "HeadPan": 0,
                "HeadTilt": -math.pi / 2,
                "LAnklePitch": 0,
                "LAnkleRoll": 0,
                "LElbow": 0,
                "LHipPitch": 0,
                "LHipRoll": 0,
                "LHipYaw": 0,
                "LKnee": 0,
                "LShoulderPitch": 0,
                "LShoulderRoll": 0,
                "RAnklePitch": 0,
                "RAnkleRoll": 0,
                "RElbow": 0,
                "RHipPitch": 0,
                "RHipRoll": 0,
                "RHipYaw": 0,
                "RKnee": 0,
                "RShoulderPitch": 0,
                "RShoulderRoll": 0,
            },
        )

        # TODO Change checks?
        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        #################### LEGS ####################

        ##################
        # Squatting-Pose #
        ##################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move into a {Style.BRIGHT}squatting pose.{Style.RESET_ALL}\n"
            f"{Fore.RED}Make sure to pick the robot up!{Style.RESET_ALL}\n\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LHipPitch": math.pi / 2,
                "LKnee": math.pi / 2,
                "RHipPitch": -math.pi / 2,
                "RKnee": -math.pi / 2,
            },
        )

        # TODO Change checks?
        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ##################
        # Men-Split-Pose #
        ##################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move into a {Style.BRIGHT}men split (front/back).{Style.RESET_ALL}\n"
            f"{Fore.RED}Make sure to pick the robot up and keep clear!{Style.RESET_ALL}\n\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LHipRoll": math.pi / 2,
                "LHipYaw": math.pi / 2,
                "RHipRoll": -math.pi / 2,
                "RHipYaw": math.pi / 2,
            },
        )

        # TODO Change checks?
        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ############################
        # Men-Split-Other-Way-Pose #
        ############################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move into a {Style.BRIGHT}men split (front/back) the other way around.{Style.RESET_ALL}\n"
            f"{Fore.RED}Make sure to pick the robot up and keep clear!{Style.RESET_ALL}\n\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LHipRoll": math.pi / 2,
                "LHipYaw": -math.pi / 2,
                "RHipRoll": -math.pi / 2,
                "RHipYaw": -math.pi / 2,
            },
        )

        # TODO Change checks?
        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ####################
        # Women-Split-Pose #
        ####################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move into a {Style.BRIGHT}women split (sides).{Style.RESET_ALL}\n"
            f"{Fore.RED}Make sure to pick the robot up and keep clear!{Style.RESET_ALL}\n\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LHipRoll": math.pi / 2,
                "RHipRoll": -math.pi / 2,
            },
        )

        # TODO Change checks?
        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both sides of the robot are symmetrical.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        # Move back to init
        move_to_joint_position(pub, {})

    except KeyboardInterrupt:
        print(f"\n\n{Fore.RED}Closing...")
    finally:
        print(Style.RESET_ALL)


if __name__ == "__main__":
    main()
