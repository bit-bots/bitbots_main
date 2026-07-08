#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Previous benchmarks were using the turtlesim launch files to compare the performance of better_launch and ROS2. However, that's not a fair comparison, as ROS2 runs the service and parameter updates in separate processes which will not appear in the memory and CPU evaluations. 
    """
    return LaunchDescription([
        Node(
            package="examples_rclpy_minimal_publisher",
            executable="publisher_local_function",
            name="my_talker",
            namespace="basic",
        ),
        Node(
            package="examples_rclpy_minimal_subscriber",
            executable="subscriber_member_function",
            name="my_listener",
            namespace="basic",
        ),
    ])
