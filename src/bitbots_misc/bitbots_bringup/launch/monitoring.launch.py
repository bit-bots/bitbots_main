#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def monitoring():
    bl = BetterLaunch()

    bl.include("udp_bridge", "send.launch")
    bl.include("system_monitor", "system_monitor.launch.py", required=False)
