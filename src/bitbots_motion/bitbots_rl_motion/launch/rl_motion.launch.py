#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def rl_motion(
    sim: bool = False,
    walk: bool = True,
    kick: bool = True,
    mjlab_walk: bool = False,
    mjlab_getup: bool = True,
):
    bl = BetterLaunch()

    if walk:
        bl.node(
            "bitbots_rl_motion",
            "walk_node",
            "walk_node",
            param_files=bl.find("bitbots_rl_motion", "playground_walk_model.yaml", "configs"),
            use_sim_time=sim,
        )

    if kick:
        bl.node(
            "bitbots_rl_motion",
            "kick_ball_node",
            "kick_ball_node",
            param_files=bl.find("bitbots_rl_motion", "kick_ball_model.yaml", "configs"),
            use_sim_time=sim,
        )

    if mjlab_walk:
        bl.node(
            "bitbots_rl_motion",
            "mjlab_walk_node",
            "mjlab_walk_node",
            param_files=bl.find("bitbots_rl_motion", "mjlab_walk_model.yaml", "configs"),
            use_sim_time=sim,
        )

    if mjlab_getup:
        bl.node(
            "bitbots_rl_motion",
            "mjlab_getup_node",
            "mjlab_getup_node",
            param_files=bl.find("bitbots_rl_motion", "mjlab_getup_model.yaml", "configs"),
            use_sim_time=sim,
        )
