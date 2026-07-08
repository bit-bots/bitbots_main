#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def system_monitor():
    bl = BetterLaunch()
    bl.node(
        "system_monitor",
        "monitor",
        "",
        param_files=bl.find("system_monitor", "config.yaml", "config"),
    )
