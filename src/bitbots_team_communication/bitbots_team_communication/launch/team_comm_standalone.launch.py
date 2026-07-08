#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def team_comm_standalone(sim: bool = False):
    """
    Parameters
    ----------
    sim : bool
        true: activates simulation time
    """
    bl = BetterLaunch()

    bl.include("bitbots_parameter_blackboard", "parameter_blackboard.launch.py")

    bl.node(
        "bitbots_team_communication",
        "team_comm.py",
        "",
        param_files=bl.find("bitbots_team_communication", "team_communication_config.yaml", "config"),
        use_sim_time=sim,
    )
