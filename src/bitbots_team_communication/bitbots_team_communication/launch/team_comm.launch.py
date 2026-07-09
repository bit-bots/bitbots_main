#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def team_comm(sim: bool = False):
    """
    Parameters
    ----------
    sim : bool
        true: activates simulation time
    """
    bl = BetterLaunch()

    bl.node(
        "bitbots_team_communication",
        "team_comm.py",
        "team_comm",
        param_files=bl.find("bitbots_team_communication", "team_communication_config.yaml", "config"),
        use_sim_time=sim,
    )
