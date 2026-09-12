#!/usr/bin/env python3
import os

from better_launch import BetterLaunch, launch_this


@launch_this
def simulator(web: bool = False):
    """
    Parameters
    ----------
    web : bool
        Use web-based mjviser viewer instead of the native MuJoCo viewer
    """
    bl = BetterLaunch()

    if web:
        # The web viewer runs headless (no X server), so MuJoCo needs to render the cameras via EGL instead of GLFW
        os.environ["MUJOCO_GL"] = "egl"

    bl.node(
        "bitbots_mujoco_sim",
        "sim",
        "sim_interface",
        params={
            "use_namespace": False,
            "web": web,
        },
    )
