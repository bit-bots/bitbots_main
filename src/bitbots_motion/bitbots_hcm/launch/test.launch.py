#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def test(sim: bool = False):
    """
    Parameters
    ----------
    sim : bool
        Disables checks for hardware, since we are in simulation.
    """
    bl = BetterLaunch()

    bl.include("bitbots_parameter_blackboard", "parameter_blackboard.launch.py", sim=sim)
    bl.include("bitbots_robot_description", "load_robot_description.launch.py", sim=sim)

    if not sim:
        bl.include("livelybot_bringup", "lowlevel.launch")

    bl.include("bitbots_animation_server", "animation.launch.py", sim=sim)
    bl.include("bitbots_hcm", "hcm.launch.py", sim=sim)
