#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def animation(sim: bool = False):
    """
    Parameters
    ----------
    sim : bool
        Disables some checks for hardware, since we are in simulation.
    """
    bl = BetterLaunch()
    bl.node(
        "bitbots_animation_server",
        "animation_node",
        "",
        use_sim_time=sim,
    )
