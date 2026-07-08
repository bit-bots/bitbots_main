#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def motion_standalone(
    sim: bool = False,
    viz: bool = False,
    torqueless_mode: bool = False,
    tts: bool = True,
):
    """
    Parameters
    ----------
    torqueless_mode : bool
        start without torque, for example for testing the falling detection
    tts : bool
        Whether to enable text-to-speech
    """
    bl = BetterLaunch()

    bl.include("bitbots_parameter_blackboard", "parameter_blackboard.launch.py", sim=sim)
    bl.include("bitbots_robot_description", "load_robot_description.launch.py", sim=sim)

    if tts:
        bl.include("bitbots_tts", "tts.launch.py")

    if viz:
        bl.node("bitbots_utils", "motor_goals_viz_helper.py", "", cmd_args=["--all"])
        bl.node("rviz2", "rviz2", "")
        bl.node("bitbots_utils", "dummy_imu.py", "")

    bl.include(
        "bitbots_bringup",
        "motion.launch.py",
        sim=sim,
        viz=viz,
        torqueless_mode=torqueless_mode,
    )
