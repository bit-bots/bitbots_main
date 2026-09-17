#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def viz():
    bl = BetterLaunch()
    bl.node(
        "plotjuggler",
        "plotjuggler",
        "plotjuggler",
        anonymous=True,
        cmd_args=["--layout", bl.find("system_monitor", "plotjuggler_layout.xml", "config")],
    )
