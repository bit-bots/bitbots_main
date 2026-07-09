#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def hcm(sim: bool = False, viz: bool = False, wolfgang: bool = True):
    """
    Parameters
    ----------
    sim : bool
        Disables some checks for hardware, since we are in simulation.
    viz : bool
        Disables all checks for hardware, since we are in visualization.
    """
    bl = BetterLaunch()

    if wolfgang:
        bl.node(
            "bitbots_hcm",
            "HCM",
            "hcm_cpp",
            params={
                "simulation_active": sim,
                "visualization_active": viz,
            },
            use_sim_time=sim,
        )
