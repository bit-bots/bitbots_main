#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def tts():
    bl = BetterLaunch()
    bl.node(
        "bitbots_tts",
        "tts",
        "bitbots_tts",
        params="tts_config.yaml",
    )
