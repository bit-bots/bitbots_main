#!/usr/bin/env python3
import logging
import os

from better_launch import BetterLaunch, launch_this


@launch_this
def parameter_blackboard(sim: bool = False, fieldname: str = None):
    """Loads the global parameters onto a parameter_blackboard node.

    Parameters
    ----------
    sim : bool
        Whether we are running in simulation.
    fieldname : str
        Field name to load parameters for. Defaults to "hsl_kid" in simulation, "labor" otherwise.
    """
    bl = BetterLaunch()

    if fieldname is None:
        fieldname = "hsl_kid" if sim else "labor"

    bl.include("bitbots_utils", "welcome.launch.py")

    param_files = [
        bl.find("bitbots_parameter_blackboard", "config.yaml", f"config/fields/{fieldname}"),
        bl.find("bitbots_parameter_blackboard", "global_parameters.yaml", "config"),
        bl.find("bitbots_parameter_blackboard", "game_settings.yaml", "config"),
    ]

    robot_domain = os.environ.get("ROS_DOMAIN_ID")
    if sim and robot_domain is not None:
        param_files.append(
            bl.find(
                "bitbots_parameter_blackboard",
                f"sim_game_settings_{int(robot_domain)}.yaml",
                "config",
            )
        )

    bl.node(
        "demo_nodes_cpp",
        "parameter_blackboard",
        "parameter_blackboard",
        params={
            "simulation_active": sim,
            "use_sim_time": sim,
            "field.name": fieldname,
        },
        param_files=param_files,
        log_level=logging.WARNING,
    )
