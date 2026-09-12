#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def vision_standalone(sim: bool = False, camera: bool = True, debug: bool = False, fieldname: str = None):
    """
    Parameters
    ----------
    sim : bool
        true: activates simulation time, switches to simulation color settings and deactivates
        launching of an image provider
    camera : bool
        true: launches an image provider to get images from a camera (unless sim:=true)
    debug : bool
        true: activates publishing of several debug images
    fieldname : str
        Loads field settings. Defaults to "small_division_2026" in simulation, "labor" otherwise.
    """
    bl = BetterLaunch()

    if fieldname is None:
        fieldname = "small_division_2026" if sim else "labor"

    # Load the global parameters
    bl.include("bitbots_parameter_blackboard", "parameter_blackboard.launch.py", sim=sim, fieldname=fieldname)

    # Load the diagnostic aggregator
    bl.include("bitbots_diagnostic", "aggregator.launch.py")

    # Start the vision
    bl.include("bitbots_bringup", "vision.launch.py", sim=sim, camera=camera, debug=debug)
