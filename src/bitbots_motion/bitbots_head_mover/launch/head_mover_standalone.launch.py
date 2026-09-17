#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def head_mover_standalone(sim: bool = False):
    bl = BetterLaunch()

    bl.include("bitbots_parameter_blackboard", "parameter_blackboard.launch.py", sim=sim)
    bl.include("bitbots_robot_description", "load_robot_description.launch.py", sim=sim)
    bl.include("bitbots_head_mover", "head_mover.launch.py", sim=sim)
