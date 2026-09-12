#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def vision(sim: bool = False, debug: bool = False):
    """
    Parameters
    ----------
    sim : bool
        true: activates simulation time
    debug : bool
        true: activates publishing of the debug image
    """
    bl = BetterLaunch()

    bl.node(
        "bitbots_vision",
        "vision",
        "bitbots_vision",
        param_files=bl.find("bitbots_vision", "visionparams.yaml", "config"),
        params={"components.debug_active": debug},
        use_sim_time=sim,
    )
