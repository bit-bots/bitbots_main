#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def test(sim: bool = False):
    bl = BetterLaunch()

    if not sim:
        bl.include("livelybot_bringup", "lowlevel.launch")

    bl.include("bitbots_parameter_blackboard", "parameter_blackboard.launch.py")
    bl.include("bitbots_robot_description", "load_robot_description.launch.py")

    bl.node("bitbots_animation_server", "animation_node", "animation_server")
    bl.node("bitbots_animation_server", "animation_hcm_bridge.py", "animation_hcm_bridge")
