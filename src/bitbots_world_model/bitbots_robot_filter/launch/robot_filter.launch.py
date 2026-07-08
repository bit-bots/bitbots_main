#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def robot_filter(sim: bool = False):
    bl = BetterLaunch()
    bl.node(
        "bitbots_robot_filter",
        "filter",
        "bitbots_robot_filter",
        param_files=bl.find("bitbots_robot_filter", "params.yaml", "config"),
        use_sim_time=sim,
    )
