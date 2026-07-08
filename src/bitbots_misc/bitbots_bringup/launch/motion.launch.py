#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def motion(
    sim: bool = False,
    viz: bool = False,
    torqueless_mode: bool = False,
    rl_motion: bool = True,
):
    """
    Parameters
    ----------
    torqueless_mode : bool
        start without torque, for example for testing the falling detection
    rl_motion : bool
        Whether to use the RL motion system instead of the regular one
    """
    bl = BetterLaunch()

    # launch the hardware interface
    if not sim:
        bl.include("livelybot_bringup", "lowlevel.launch")

    # launch the base footprint
    bl.node(
        "humanoid_base_footprint",
        "base_footprint",
        "",
        params={"support_state_topics": ["walk_support_state"]},
        use_sim_time=sim,
    )

    # launch the odometry
    bl.include("bitbots_odometry", "odometry.launch.py", sim=sim)

    # launch the animation server
    bl.include("bitbots_animation_server", "animation.launch.py", sim=sim)

    # launch the head mover
    bl.include("bitbots_head_mover", "head_mover.launch.py", sim=sim)

    if rl_motion:
        bl.include("bitbots_rl_motion", "rl_motion.launch.py", sim=sim)

    # launch the hcm
    bl.include("bitbots_hcm", "hcm.launch.py", sim=sim, viz=viz)
