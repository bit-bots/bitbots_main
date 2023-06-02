from typing import List

import os

from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    '/debug_markers',
    '/debug/approach_point',
    '/debug/ball_twist',
    '/debug/dsd/body_behavior',
    '/debug/dsd/hcm',
    '/debug/dsd/head_behavior',
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
    '/line_mask_relative_pc_to_record',
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


def generate_launch_description():
    configuration_sim = LaunchConfiguration('sim')

    arg_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true: Use simulation time',
        choices=['true', 'false']
    )

    arg_max_image_frequency = DeclareLaunchArgument(
        'max_image_frequency',
        default_value='1.0',
        description='Max frequency [hz] for recording images'
    )

    arg_max_pointcloud_frequency = DeclareLaunchArgument(
        'max_pointcloud_frequency',
        default_value='1.0',
        description='Max frequency [hz] for recording pointclouds'
    )

    node_drop_images = Node(
        package='topic_tools',
        executable='throttle',
        output='screen',
        name='record_rosbag_drop_images',
        arguments=[
            'messages',
            '/camera/image_proc',
            LaunchConfiguration('max_image_frequency'),
            '/camera/image_to_record'
        ]
    )
    
    node_drop_pointclouds = Node(
        package='topic_tools',
        executable='throttle',
        output='screen',
        name='record_rosbag_drop_pointclouds',
        arguments=[
            'messages',
            '/line_mask_relative_pc',
            LaunchConfiguration('max_pointcloud_frequency'),
            '/line_mask_relative_pc_to_record'
        ]
    )

    ####################################################
    # NOTE:
    # The following main process code is duplicated with a minor change in
    # the condition (sim vs. !sim) and the use of the --use-sim-time option.
    # IF YOU CHANGE SOMETHING, CHANGE IT IN BOTH PLACES!
    # 
    # The duplication is due to the fact that is seems impossible to
    # set the --use-sim-time option inplace conditionally on the launch argument 'sim'.
    ####################################################

    main_process_unless_sim = ExecuteProcess(
        condition=IfCondition(NotEqualsSubstitution(configuration_sim, 'true')),
        # Constructing the complete command
        cmd=[
                # Main command to start recording ros2 bags
                'ros2',
                'bag',
                'record',

                # Set output directory
                # ~/rosbags/ID_<robot_id>_<datetime>
                '-o', PathJoinSubstitution([
                    EnvironmentVariable('HOME'),
                    'rosbags',
                    'ID_' + 
                    os.getenv(
                        'ROBOCUP_ROBOT_ID',
                        default=os.getenv(
                            'ROBOT_NAME',
                            default='unknown_robot')) +
                    '_' +
                    datetime.now().isoformat(timespec='seconds')
                ]),

                # Other options
                '--node-name', 'ros2_bag_record',
                '--include-hidden-topics',
                '--include-unpublished-topics',
                '--polling-interval', '1000',
                [f'{topic} ' for topic in TOPICS_TO_RECORD]
        ],
        output='screen',
        name='launch_ros2_bag_record_unless_sim',
        shell=True
    )

    main_process_if_sim = ExecuteProcess(
        condition=IfCondition(EqualsSubstitution(configuration_sim, 'true')),
        # Constructing the complete command
        cmd=[
                # Main command to start recording ros2 bags
                'ros2',
                'bag',
                'record',

                # Set output directory
                # ~/rosbags/ID_<robot_id>_<datetime>
                '-o', PathJoinSubstitution([
                    EnvironmentVariable('HOME'),
                    'rosbags',
                    'ID_' + 
                    os.getenv('ROBOCUP_ROBOT_ID', default=os.getenv('ROBOT_NAME', default='unknown_robot')) +
                    '_' +
                    datetime.now().isoformat(timespec='seconds')
                ]),

                # Other options
                '--node-name', 'ros2_bag_record',
                '--include-hidden-topics',
                '--include-unpublished-topics',
                '--polling-interval', '1000',
                '--use-sim-time',
                [f'{topic} ' for topic in TOPICS_TO_RECORD]
        ],
        output='screen',
        name='launch_ros2_bag_record_if_sim',
        shell=True
    )

    ####################################################
    # NOTE:
    # The above main process code is duplicated with a minor change in
    # the condition (sim vs. !sim) and the use of the --use-sim-time option.
    # IF YOU CHANGE SOMETHING, CHANGE IT IN BOTH PLACES!
    # 
    # The duplication is due to the fact that is seems impossible to
    # set the --use-sim-time option inplace conditionally on the launch argument 'sim'.
    ####################################################

    # Construct LaunchDescription from parts
    return LaunchDescription([
        arg_sim,
        arg_max_image_frequency,
        arg_max_pointcloud_frequency,
        node_drop_images,
        node_drop_pointclouds,
        main_process_unless_sim,
        main_process_if_sim
    ])
