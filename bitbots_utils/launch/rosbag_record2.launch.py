from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='false', description='true: Use simulation time'),
        DeclareLaunchArgument(
            'max_image_frequency', default_value='1.0', description='Max frequency [hz] for recording images'),
        DeclareLaunchArgument(
            'max_pointcloud_frequency', default_value='1.0', description='Max frequency [hz] for recording pointclouds'),

        Node(
            package='topic_tools', executable='throttle', output='screen',
            name='record_rosbag_drop_images',
            arguments=[
                'messages', '/camera/image_proc',
                LaunchConfiguration('max_image_frequency'),
                '/camera/image_to_record'
            ]
        ),
        Node(
            package='topic_tools', executable='throttle', output='screen',
            name='record_rosbag_drop_pointclouds',
            arguments=[
                'messages', '/line_mask_relative_pc',
                LaunchConfiguration('max_pointcloud_frequency'),
                '/line_mask_relative_pc_to_record'
            ]
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', EnvironmentVariable('HOME') + '/rosbags/ID_' +
                EnvironmentVariable('ROBOCUP_ROBOT_ID', default_value=EnvironmentVariable('ROBOT_NAME', default_value='unknown_robot')) +
                '_' + Command(['bash', '-c', '"date +%s |tr -d \\"\\n\\"']),
                '--node-name', 'ros2_bag_record',
                '--include-hidden-topics',
                '--include-unpublished-topics',
                '--polling-interval', '1000',
                '--use_sim_time' if LaunchConfiguration('sim')=='true' else '--include-hidden-topics',
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
                '/time_to_ball'
            ],
            output='screen'
        ),
    ])
