#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def localization(tf_prefix: str = "", sim: bool = False):
    """
    Parameters
    ----------
    sim : bool
        true: activates simulation time and might load different parameters
    """
    bl = BetterLaunch()

    bl.node(
        "bitbots_localization",
        "localization",
        "bitbots_localization",
        param_files=bl.find("bitbots_localization", "config.yaml", "config"),
        params={
            "ros.odom_frame": f"{tf_prefix}odom",
            "ros.base_footprint_frame": f"{tf_prefix}base_footprint",
            "ros.map_frame": f"{tf_prefix}map",
            "ros.publishing_frame": f"{tf_prefix}localization_raw",
        },
        use_sim_time=sim,
    )

    bl.node(
        "bitbots_localization_handler",
        "localization_handler",
        "bitbots_localization_handler",
        params={
            "odom_frame": f"{tf_prefix}odom",
            "base_footprint_frame": f"{tf_prefix}base_footprint",
            "walking_moved_distance": 0.5,
        },
        use_sim_time=sim,
    )
