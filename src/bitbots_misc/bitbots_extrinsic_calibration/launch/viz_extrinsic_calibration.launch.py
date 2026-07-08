#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def viz_extrinsic_calibration():
    """Launch teamplayer with only the necessary components, plus rviz and rqt_reconfigure for extrinsic calibration."""
    bl = BetterLaunch()

    bl.include(
        "bitbots_bringup",
        "teamplayer.launch.py",
        game_controller=False,
        behavior=False,
        path_planning=False,
        world_model=False,
        teamcom=False,
        monitoring=False,
        record=False,
    )

    bl.node(
        "rviz2",
        "rviz2",
        "extrinsic_calibration_rviz",
        cmd_args=["-d", bl.find("bitbots_extrinsic_calibration", "extrinsic_calibration.rviz")],
        log_level=None,
    )

    bl.node("rqt_reconfigure", "rqt_reconfigure", "rqt_reconfigure")

    bl.process(
        ["ros2", "topic", "pub", "--once", "/head_mode", "bitbots_msgs/msg/HeadMode", "{head_mode: 1}"],
        name="set_headmode",
        output="screen",
    )
