#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def whistle_detector():
    bl = BetterLaunch()
    bl.node("bitbots_whistle_detector", "whistle_detector", "bitbots_whistle_detector")
