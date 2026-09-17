#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def robot_teleop(type: str = "noname", head: bool = False):
    """
    Parameters
    ----------
    type : str
        Sets the controller type e.g. noname, xbox
    """
    bl = BetterLaunch()

    bl.node(
        "joy_linux",
        "joy_linux_node",
        "joy_node",
        params={
            "deadzone": 0.1,
            "autorepeat_rate": 10.0,
        },
    )

    bl.node(
        "bitbots_teleop",
        "joy_node",
        "joy_to_twist",
        params={
            "type": type,
            "head": head,
        },
        param_files=bl.find("bitbots_teleop", "controller.yaml", "config"),
    )
