#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def demo(sim: bool = False, behavior_dsd_file: str = "demo.dsd"):
    """
    Parameters
    ----------
    sim : bool
        Whether the robot is running in simulation or on real hardware
    behavior_dsd_file : str
        The behavior dsd file that should be used
    """
    bl = BetterLaunch()

    # load teamplayer software stack without some unnecessary stuff, that is not needed in the demo
    bl.include(
        "bitbots_bringup",
        "teamplayer.launch.py",
        behavior_dsd_file=behavior_dsd_file,
        fieldname="demo",
        game_controller=False,
        localization=False,
        sim=sim,
        teamcom=False,
        path_planning=True,
    )
