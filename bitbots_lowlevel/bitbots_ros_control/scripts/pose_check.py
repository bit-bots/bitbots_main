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

SPEED: float = 1.0
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
    return f"{''.join([emoji_numbers[int(digit)] for digit in str(num)])} "


def move_to_joint_position(publisher: Publisher, joint_goals: dict[str, float], offsets=True):
    assert set(joint_goals.keys()) <= set(JOINT_NAMES), "Joint goals do not match the robots joints"

    # Init all joints with 0
    all_joint_goals = {joint: 0.0 for joint in JOINT_NAMES}
    all_joint_goals.update(joint_goals)

    if offsets:
        # offsets for straight legs
        all_joint_goals["LHipPitch"] -= 0.327
        all_joint_goals["RHipPitch"] += 0.327
        all_joint_goals["LKnee"] -= 0.097
        all_joint_goals["RKnee"] += 0.097
        all_joint_goals["LAnklePitch"] += 0.084
        all_joint_goals["RAnklePitch"] -= 0.084

    publisher.publish(
        JointCommand(
            joint_names=all_joint_goals.keys(),
            velocities=[SPEED] * len(JOINT_NAMES),
            accelerations=[-1.0] * len(JOINT_NAMES),
            max_currents=[-1.0] * len(JOINT_NAMES),
            positions=all_joint_goals.values(),
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

    # Show bitbot ascii art (TODO change to T-Posing Bit-Bot)
    ascii_art = open(os.path.join(get_package_share_directory("bitbots_utils"), "config", "welcome_art.txt")).read()

    print(
        f"\n\n{ascii_art}\n\n"
        f"\n{Fore.LIGHTWHITE_EX}{Style.BRIGHT}{Back.BLUE}                                                {Style.RESET_ALL}\n"
        f"{Fore.LIGHTWHITE_EX}{Style.BRIGHT}{Back.BLUE}    We now start with the calibration check!    {Style.RESET_ALL}\n"
        f"{Fore.LIGHTWHITE_EX}{Style.BRIGHT}{Back.BLUE}    ========================================    {Style.RESET_ALL}\n"
        f"{Fore.LIGHTWHITE_EX}{Style.BRIGHT}{Back.BLUE}                                                {Style.RESET_ALL}\n\n"
        f"{Fore.RED}Make sure that the surroundings are clear and the robot is able to move freely. "
        f"Also be cautious and prepared to hold the robot in case it is not stable.\n{Style.RESET_ALL}"
    )

    try:
        input(
            f"{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move into a {Style.BRIGHT}t-pose.{Style.RESET_ALL} 🥵\n\n"
            f"{Style.BRIGHT}Press enter to move."
        )
        #################### ARMS and HEAD ####################

        ##########
        # T-Pose #
        ##########

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
            "- Check for backlash and loose parts.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ##########################
        # Arms-To-The-Front-Pose #
        ##########################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move {Style.BRIGHT}its arms to the front, its elbows to a 90 degree angle, and its headto the left.{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LShoulderPitch": -math.pi / 2,
                "RShoulderPitch": math.pi / 2,
                "HeadPan": math.pi / 2,
            },
        )

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that the arms point to the front.\n"
            "- Check that the elbows are at 90 degrees.\n"
            "- Check that both arms are symmetrical.\n"
            "- Check that the head looks 90 degrees to the left.\n"
            "- Check that the head is horizontal.\n"
            "- Check for backlash and loose parts.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ##################
        # Arms-Down-Pose #
        ##################

        step += 1
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move {Style.BRIGHT}its arms and its head down.{Style.RESET_ALL}\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LElbow": -math.pi / 2,
                "RElbow": math.pi / 2,
                "HeadTilt": -math.pi / 2,
            },
        )

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that all limbs are straight.\n"
            "- Check that both arms are symmetrical.\n"
            "- Check that the head is oriented to the front.\n"
            "- Check that the head looks straight down in reference to the torso.\n"
            "- Check for backlash and loose parts.\n"
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

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that both sides of the robot are symmetrical.\n"
            "- Check that the knees are at 90 degrees.\n"
            "- Check for backlash or flexibility of the legs.\n"
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

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that the legs are straight.\n"
            "- Check if the legs moved freely to the position (sometimes the hipYaw screws are tightened too much and block the range of motion).\n"
            "- Check that the legs are not flexible by an unusual amount if moved.\n"
            "- Check that the cables are without strain.\n"
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

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that the legs are straight.\n"
            "- Check if the legs moved freely to the position (sometimes the hipYaw screws are tightened too much and block the range of motion).\n"
            "- Check that the legs are not flexible by an unusual amount if moved.\n"
            "- Check that the cables are without strain.\n"
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

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            "- Check that the legs are symmetrical.\n"
            "- Check that the legs are straight.\n"
            "- Check that the legs are horizontal.\n"
            "- Check that the legs are not flexible by an unusual amount if moved.\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        ###########
        # Ankles #
        ##########

        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move {Style.BRIGHT}back and lift its feet.{Style.RESET_ALL}\n"
            f"{Fore.RED}Make sure to pick the robot up and keep clear!{Style.RESET_ALL}\n\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LAnklePitch": -math.pi / 4,
                "RAnklePitch": math.pi / 4,
            },
        )

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            f"- Check that the feet are lifted at a 45 degree angle\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )
        input(
            f"\n{Fore.YELLOW}{num_to_emoji(step)}: The robot will now move its {Style.BRIGHT}ankles.{Style.RESET_ALL}\n"
            f"{Fore.RED}Make sure to pick the robot up and keep clear!{Style.RESET_ALL}\n\n"
            f"{Style.BRIGHT}Press enter to move.{Style.RESET_ALL}"
        )

        move_to_joint_position(
            pub,
            {
                "LAnkleRoll": -math.pi / 2,
                "RAnkleRoll": math.pi / 2,
            },
        )

        input(
            f"{Style.BRIGHT}\nChecks:{Style.RESET_ALL}\n"
            f"- Check that the feet are rolled to the side at a 90 degree angle\n"
            f"{Style.BRIGHT}Press enter to continue.{Style.RESET_ALL}"
        )

        move_to_joint_position(pub, {})
        print(
            f"\n\n{Fore.GREEN}If you have reached this point, the robot is hopefully in good shape and ready to go! GLHF"
        )

    except KeyboardInterrupt:
        print(f"\n\n{Fore.RED}Closing...")
    finally:
        print(Style.RESET_ALL)


if __name__ == "__main__":
    main()
