#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def team_comm_test_marker(rviz: bool = True):
    bl = BetterLaunch()

    if rviz:
        bl.node(
            "rviz2",
            "rviz2",
            "",
            cmd_args=["-d", bl.find("bitbots_team_communication", "team_comm_marker.rviz", "config")],
            log_level=None,
        )

    bl.node(
        "bitbots_team_communication",
        "team_comm_test_marker.py",
        "team_comm_test_marker",
        param_files=bl.find("bitbots_team_communication", "team_communication_config.yaml", "config"),
    )
