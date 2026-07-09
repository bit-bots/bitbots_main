#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def emergency_listener(emergency_button: bool = True):
    bl = BetterLaunch()

    if emergency_button:
        bl.node("bitbots_emergency", "EMERGENCY_NODE_LISTENER", "emergency_node_listener")
