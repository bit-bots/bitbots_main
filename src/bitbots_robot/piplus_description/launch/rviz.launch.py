#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def rviz():
    bl = BetterLaunch()
    bl.node(
        "rviz2",
        "rviz2",
        "rviz2",
        cmd_args=["-d", bl.find("piplus_description", "piplus.rviz", "config")],
        log_level=None,
    )
