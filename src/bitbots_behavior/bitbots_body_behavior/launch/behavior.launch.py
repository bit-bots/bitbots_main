#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def behavior(dsd_file: str = "main.dsd", tf_prefix: str = "", sim: bool = False):
    bl = BetterLaunch()

    bl.node(
        "bitbots_body_behavior",
        "body_behavior",
        "",
        params={
            "dsd_file": dsd_file,
            "actionlib_server_sub_queue_size": -1,
            "odom_frame": f"{tf_prefix}odom",
            "map_frame": f"{tf_prefix}map",
            "base_footprint_frame": f"{tf_prefix}base_footprint",
        },
        param_files=[
            bl.find("bitbots_body_behavior", "body_behavior.yaml", "config"),
            bl.find("bitbots_body_behavior", "animations.yaml", "config"),
        ],
        use_sim_time=sim,
        max_respawns=-1,
    )
