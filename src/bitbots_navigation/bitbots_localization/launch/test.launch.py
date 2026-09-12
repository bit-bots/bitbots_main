#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def test():
    bl = BetterLaunch()

    bl.include(
        "bitbots_bringup",
        "simulator_teamplayer.launch.py",
        behavior=False,
        game_controller=False,
        localization=True,
    )

    bl.node(
        "rviz2",
        "rviz2",
        "rviz",
        anonymous=True,
        cmd_args=["-d", bl.find("bitbots_localization", "localization.rviz", "config")],
        log_level=None,
    )
