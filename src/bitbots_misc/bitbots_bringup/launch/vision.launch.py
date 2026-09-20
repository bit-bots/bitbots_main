#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def vision(sim: bool = False, camera: bool = True, debug: bool = False):
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
    """
    bl = BetterLaunch()

    # Start the vision
    bl.include("bitbots_vision", "vision.launch.py", sim=sim, debug=debug)

    # Start the camera only when necessary
    if camera and not sim:
        bl.include(
            "zed_wrapper",
            "zed_camera.launch.py",
            camera_model="zedm",
            publish_urdf=False,
        )
