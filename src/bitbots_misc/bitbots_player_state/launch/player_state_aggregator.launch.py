#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def player_state_aggregator(sim: bool = False, config: str = None):
    """
    Parameters
    ----------
    sim : bool
        Whether to use simulation time
    config : str
        Player state aggregator configuration file
    """
    bl = BetterLaunch()

    if config is None:
        config = bl.find("bitbots_player_state", "player_state_aggregator.yaml", "config")

    bl.node(
        "bitbots_player_state",
        "player_state_aggregator",
        "player_state_aggregator",
        param_files=config,
        use_sim_time=sim,
    )
