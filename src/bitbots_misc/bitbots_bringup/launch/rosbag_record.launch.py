#!/usr/bin/env python3
import os
from datetime import datetime

from better_launch import BetterLaunch, launch_this

TOPICS_TO_RECORD: list[str] = [
    "/animation",
    "/audio",
    "/ball_obstacle_active",
    "/ball_position_relative_filtered",
    "/balls_relative",
    "/zed/zed_node/rgb/camera_info",
    "/zed/zed_node/rgb/image_rect_color",
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
    "/joint_command",
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
    "/whistle_detected",
]


@launch_this
def rosbag_record(sim: bool = False, max_image_frequency: float = 1.0):
    """
    Parameters
    ----------
    sim : bool
        Use simulation time
    max_image_frequency : float
        Max frequency [hz] for recording images
    """
    bl = BetterLaunch()

    robot_name = os.getenv("ROBOCUP_ROBOT_ID", os.getenv("ROBOT_NAME", "unknown_robot"))

    # Set output directory
    # ~/rosbags/ID_<robot_id>_<datetime>
    output_directory = os.path.join(
        os.environ["HOME"],
        "rosbags",
        f"ID_{robot_name}_{datetime.now().isoformat(timespec='seconds')}",
    )

    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        output_directory,
        "--node-name",
        "ros2_bag_record",
        "--include-hidden-topics",
        "--include-unpublished-topics",
        "--polling-interval",
        "1000",
    ]

    if sim:
        cmd.append("--use-sim-time")

    cmd.extend(TOPICS_TO_RECORD)

    bl.process(cmd, name="ros2_bag_record", output="screen", use_shell=True)
