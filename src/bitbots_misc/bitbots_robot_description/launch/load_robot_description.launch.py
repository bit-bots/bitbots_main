#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this
from better_launch.convenience import read_robot_description, robot_state_publisher


@launch_this
def load_robot_description(sim: bool = False):
    bl = BetterLaunch()

    description = read_robot_description("piplus_description", "pi_plus_22dof.urdf.xacro", "urdf")
    robot_state_publisher(
        description,
        node_name="robot_state_publisher",
        anonymous=False,
        params={"publish_frequency": 100.0},
    )

    bl.include("bitbots_extrinsic_calibration", "calibration.launch.py", sim=sim)
