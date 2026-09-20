#!/usr/bin/env python3
import os

from better_launch import BetterLaunch, launch_this


@launch_this
def path_planning(sim: bool = False):
    """
    Parameters
    ----------
    sim : bool
        true: activates simulation time
    """
    bl = BetterLaunch()

    config_file = f"path_planning_parameters_{os.environ.get('ROBOT_NAME', 'default')}.yaml"

    bl.node(
        "bitbots_path_planning",
        "path_planning",
        "bitbots_path_planning",
        param_files=bl.find("bitbots_path_planning", config_file, "config"),
        use_sim_time=sim,
    )
