#!/usr/bin/env python3

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Hnnnngggggggg.....
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("better_launch"),
                                "examples",
                                "01_basic_example.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            Node(
                package="examples_rclpy_minimal_subscriber",
                executable="subscriber_member_function",
                name="my_listener_ROS",
                namespace="basic",
            ),
        ]
    )
