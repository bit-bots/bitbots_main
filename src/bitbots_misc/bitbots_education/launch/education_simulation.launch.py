#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def education_simulation():
    bl = BetterLaunch()

    bl.include("bitbots_education", "education.launch.py")
    bl.include("bitbots_mujoco_sim", "simulator.launch.py")
    bl.include("bitbots_bringup", "vision.launch.py", sim=True, camera=False)
    bl.include("bitbots_bringup", "motion_standalone.launch.py", sim=True)
