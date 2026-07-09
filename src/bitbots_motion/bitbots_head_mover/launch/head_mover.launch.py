#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def head_mover(tf_prefix: str = "", sim: bool = False):
    bl = BetterLaunch()
    bl.node(
        "bitbots_head_mover",
        "move_head",
        "head_mover",
        use_sim_time=sim,
        max_respawns=-1,
    )
