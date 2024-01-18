import os
import subprocess
import time

import psutil
import rclpy
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, TextSubstitution, LaunchConfiguration



def generate_launch_description():
    gui_conf = LaunchConfiguration("gui")
    headless_conf = LaunchConfiguration("headless")
    num_robots_conf = LaunchConfiguration("num_robots")
    imu_filter_conf = LaunchConfiguration("imu_filter")
    multi_robot_conf = LaunchConfiguration("multi_robot")
    sim_id_conf = LaunchConfiguration("sim_id")
    camera_conf = LaunchConfiguration("camera")
    recognition_conf = LaunchConfiguration("recognition")
    void_controller = LaunchConfiguration("void_controller")

    gui = DeclareLaunchArgument('gui', default_value='True')
    headless = DeclareLaunchArgument('headless', default_value='False')
    num_robots = DeclareLaunchArgument('num_robots', default_value='4')
    imu_filter = DeclareLaunchArgument('imu_filter', default_value='True')
    multi_robot = DeclareLaunchArgument('multi_robot', default_value='False')
    sim_id = DeclareLaunchArgument('sim_id', default_value='')
    camera = DeclareLaunchArgument('camera', default_value='True')
    recognition = DeclareLaunchArgument('recognition', default_value='False')
    void_controller = DeclareLaunchArgument('void_controller', default_value='False')

    sim_node = Node(
        package='wolfgang_webots_sim',
        namespace='',
        executable='start.py',
        name='simulation',
        arguments=[str(gui), str(multi_robot), str(headless), str(void_controller), str(recognition), str(not camera)]
     )

    return LaunchDescription([gui, headless, num_robots, imu_filter, multi_robot, sim_id, camera, recognition, sim_node])
