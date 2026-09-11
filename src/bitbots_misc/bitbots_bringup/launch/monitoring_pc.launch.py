#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def monitoring_pc():
    bl = BetterLaunch()

    # start udp bridge client to listen to the robot
    bl.include("udp_bridge", "receive.launch")

    # start foxglove bridge
    bl.include("foxglove_bridge", "foxglove_bridge_launch.xml")

    # start foxglove gui
    bl.process(["lichtblick", "--no-sandbox"], name="lichtblick", output="screen")

    # start dynamic_stack_decider_visualization dsd_gui
    bl.node("dynamic_stack_decider_visualization", "dsd_gui", "dsd_gui")
