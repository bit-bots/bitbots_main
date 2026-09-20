#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def ball_filter(sim: bool = False):
    bl = BetterLaunch()
    bl.node(
        "bitbots_ball_filter",
        "ball_filter",
        "bitbots_ball_filter",
        use_sim_time=sim,
    )
