#!/usr/bin/env python3
import os

from better_launch import BetterLaunch, launch_this


@launch_this
def odometry(sim: bool = False):
    bl = BetterLaunch()

    tf_prefix = os.environ.get("ROS_NAMESPACE", "")
    config_file = f"odometry_config_{os.environ.get('ROBOT_NAME', 'default')}.yaml"

    bl.node(
        "bitbots_odometry",
        "motion_odometry",
        "motion_odometry",
        param_files=bl.find("bitbots_odometry", config_file, "config"),
        params={
            "base_link_frame": f"{tf_prefix}base_link",
            "r_sole_frame": f"{tf_prefix}r_sole",
            "l_sole_frame": f"{tf_prefix}l_sole",
            "odom_frame": f"{tf_prefix}odom",
        },
        use_sim_time=sim,
    )
