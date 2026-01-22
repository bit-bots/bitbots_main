import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

# from launch_ros.actions import Node

TOPICS_TO_RECORD: list[str] = [
    "/animation",
    "/audio/audio_info",
    "/audio/audio",
    "/ball_obstacle_active",
    "/ball_position_relative_filtered",
    "/balls_relative",
    "/camera/camera_info",
    "/camera/image_proc",
    "/clock",
    "/cmd_vel",
    "/cop_l",
    "/cop_r",
    "/core/power_switch_status",
    "/debug/approach_point",
    "/debug/dsd/body_behavior/dsd_current_action",
    "/debug/dsd/body_behavior/dsd_stack",
    "/debug/dsd/body_behavior/dsd_tree",
    "/debug/dsd/hcm/dsd_current_action",
    "/debug/dsd/hcm/dsd_stack",
    "/debug/dsd/hcm/dsd_tree",
    "/debug/dsd/localization/dsd_current_action",
    "/debug/dsd/localization/dsd_stack",
    "/debug/dsd/localization/dsd_tree",
    "/debug/behavior/used_ball",
    "/debug/behavior/which_ball_is_used",
    "/diagnostics_agg",
    "/diagnostics",
    "/DynamixelController/command",
    "/field_boundary_relative",
    "/field/map",
    "/game_controller_connected",
    "/gamestate",
    "/goal_pose",
    "/head_mode",
    "/imu_head/data",
    "/imu/data",
    "/joint_states",
    "/motion_odometry",
    "/move_base/current_goal",
    "/path",
    "/pose_with_covariance",
    "/robot_state",
    "/robots_relative_filtered",
    "/robots_relative",
    "/rosout",
    "/server_time_clock",
    "/speak",
    "/strategy",
    "/system_workload",
    "/team_data",
    "/tf_static",
    "/tf",
    "/time_to_ball",
    "/workspace_status",
]


def generate_launch_arguments():
    return [
        DeclareLaunchArgument(
            "sim", default_value="false", description="true: Use simulation time", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "max_image_frequency", default_value="1.0", description="Max frequency [hz] for recording images"
        ),
    ]


def generate_nodes():
    return [
        # Node(
        #     package="topic_tools",
        #     executable="throttle",
        #     output="screen",
        #     name="record_rosbag_drop_images",
        #     arguments=[
        #         "messages",
        #         "/camera/image_proc",
        #         LaunchConfiguration("max_image_frequency"),
        #         "/camera/image_to_record",
        #     ],
        # )
    ]


def generate_action(context):
    robot_name = os.getenv("ROBOCUP_ROBOT_ID", default=os.getenv("ROBOT_NAME", default="unknown_robot"))

    # Set output directory
    # ~/rosbags/ID_<robot_id>_<datetime>
    output_directory = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "rosbags",
            "ID_" + robot_name + "_" + datetime.now().isoformat(timespec="seconds"),
        ]
    )

    sim_value = LaunchConfiguration("sim").perform(context)
    sim_time = ["--use-sim-time"] if sim_value == "true" else []

    node_name = "ros2_bag_record"

    main_process = ExecuteProcess(
        # Constructing the complete command
        cmd=[
            # Main command to start recording ros2 bags
            "ros2",
            "bag",
            "record",
            "-o",
            output_directory,
            # Other options
            "--node-name",
            node_name,
            "--include-hidden-topics",
            "--include-unpublished-topics",
            "--polling-interval",
            "1000",
        ]
        + sim_time
        + TOPICS_TO_RECORD,
        output="screen",
        name=node_name,
        shell=True,
    )
    return [main_process]


def generate_launch_description():
    launch_arguments = generate_launch_arguments()
    nodes = generate_nodes()

    action = OpaqueFunction(function=generate_action)

    # Construct LaunchDescription from parts
    return LaunchDescription(launch_arguments + nodes + [action])
