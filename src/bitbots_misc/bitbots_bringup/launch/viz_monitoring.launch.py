#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def viz_monitoring():
    bl = BetterLaunch()
    bl.node(
        "rviz2",
        "rviz2",
        "monitoring_rviz",
        cmd_args=["-d", bl.find("bitbots_bringup", "monitoring.rviz", "config")],
        log_level=None,
    )
