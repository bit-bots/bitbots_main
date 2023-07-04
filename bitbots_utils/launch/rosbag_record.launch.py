from typing import List

import os

from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, EqualsSubstitution, LaunchConfiguration, NotEqualsSubstitution, PathJoinSubstitution
from launch_ros.actions import Node


TOPICS_TO_RECORD: List[str] = [
    '/animation',
    '/audio/audio_info',
    '/audio/audio',
    '/ball_obstacle_active',
    '/ball_position_relative_filtered',
    '/ball_relative_filtered',
    '/ball_relative_movement',
    '/balls_relative',
    '/camera/camera_info',
    '/camera/image_to_record',
    '/clock',
    '/cmd_vel',
    '/cop_l',
    '/cop_r',
    '/core/power_switch_status',
    '/debug/approach_point',
    '/debug/ball_twist',
    '/debug/dsd/body_behavior',
    '/debug/dsd/hcm',
    '/debug/dsd/localization',
    '/debug/used_ball',
    '/debug/which_ball_is_used',
    '/diagnostics',
    '/diagnostics_agg',
    '/DynamixelController/command',
    '/field_boundary_relative',
    '/game_controller_connected',
    '/gamestate',
    '/goal_pose',
    '/head_mode',
    '/imu_head/data',
    '/imu/data_raw',
    '/joint_states',
    '/motion_odometry',
    '/move_base/current_goal',
    '/obstacles_relative',
    '/pose_with_covariance',
    '/robot_state',
    '/robots_relative',
    '/robots_relative_filtered',
    '/rosout',
    '/server_time_clock',
    '/speak',
    '/strategy',
    '/system_workload',
    '/team_data',
    '/tf',
    '/tf_static',
    '/time_to_ball',
]


def generate_launch_arguments():
    return [
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='true: Use simulation time',
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'max_image_frequency',
            default_value='1.0',
            description='Max frequency [hz] for recording images'
        ),
        DeclareLaunchArgument(
            'max_pointcloud_frequency',
            default_value='1.0',
            description='Max frequency [hz] for recording pointclouds'
        ),
    ]


def generate_action(context):
    robot_name = os.getenv('ROBOCUP_ROBOT_ID', default=os.getenv('ROBOT_NAME', default='unknown_robot'))

    # Set output directory
    # ~/rosbags/ID_<robot_id>_<datetime>
    output_directory = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'rosbags',
        'ID_' + robot_name + '_' + datetime.now().isoformat(timespec='seconds')
    ])

    sim_value = LaunchConfiguration('sim').perform(context)
    sim_time = ['--use-sim-time'] if sim_value == 'true' else []

    main_process = ExecuteProcess(
        # Constructing the complete command
        cmd=[
            # Main command to start recording ros2 bags
            'ros2',
            'bag',
            'record',
            '-o', output_directory,

            # Other options
            '--node-name', 'ros2_bag_record',
            '--include-hidden-topics',
            '--include-unpublished-topics',
            '--polling-interval', '1000',
        ] + sim_time + TOPICS_TO_RECORD,
        output='screen',
        name='ros2_bag_record',
        shell=True
    )
    return [main_process]


def generate_launch_description():
    launch_arguments = generate_launch_arguments()

    action = OpaqueFunction(function=generate_action)

    # Construct LaunchDescription from parts
    return LaunchDescription(launch_arguments + [action])
