#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def behavior_standalone(sim: bool = False, dsd_file: str = "main.dsd"):
    """
    Parameters
    ----------
    dsd_file : str
        The behavior dsd file that should be used
    """
    bl = BetterLaunch()

    fieldname = "small_division_2026" if sim else "labor"

    bl.include(
        "bitbots_parameter_blackboard",
        "parameter_blackboard.launch.py",
        sim=sim,
        fieldname=fieldname,
    )

    bl.include(
        "bitbots_body_behavior",
        "behavior.launch.py",
        dsd_file=dsd_file,
        sim=sim,
    )
