#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def emergency_publisher(emergency_button: bool = True, robot_ip: str = "10.10.10.10"):
    bl = BetterLaunch()

    if emergency_button:
        script = bl.find("bitbots_emergency", "publisher.sh", "scripts")
        bl.process(["bash", script, robot_ip], name="publisher", output="screen")
