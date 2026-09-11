#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def whistle_detector(sim: bool = False):
    bl = BetterLaunch()
    bl.node("bitbots_whistle_detector", "whistle_detector", "bitbots_whistle_detector", params={"use_sim_time": sim})
