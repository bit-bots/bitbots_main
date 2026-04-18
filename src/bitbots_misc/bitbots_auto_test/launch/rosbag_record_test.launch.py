from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution

# from launch_ros.actions import Node

TOPICS_TO_RECORD: list[str] = [
    "/animation",
    "/ball_obstacle_active",
    "/ball_position_relative_filtered",
    "/balls_relative",
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
        DeclareLaunchArgument("run_name", default_value="", description="Name used for log files, animation, and rosbag output"),
    ]


def generate_nodes():
    return []


def generate_action(context):
    run_name = context.launch_configurations.get("run_name", "").strip()
    if not run_name:
        run_name = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")

    # Set output directory to ~/monitoring_logs/<run_name>.rosbag
    output_directory = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "monitoring_logs",
            run_name + ".rosbag",
        ]
    )

    node_name = "auto_test_record"

    main_process = ExecuteProcess(
        # Constructing the complete command
        cmd=[
            # Main command to start recording ros2 bags
            "python",
            FindExecutable(name="ros2"),
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
            "--use-sim-time",
            "--topics"
        ]
        + TOPICS_TO_RECORD,
        output="screen",
        name=node_name,
        shell=False,
    )
    return [main_process]


def generate_launch_description():
    launch_arguments = generate_launch_arguments()
    nodes = generate_nodes()

    action = OpaqueFunction(function=generate_action)

    # Construct LaunchDescription from parts
    return LaunchDescription(launch_arguments + nodes + [action])
