#!/usr/bin/env python3
import os

from better_launch import BetterLaunch, launch_this


@launch_this
def calibration(sim: bool = False):
    bl = BetterLaunch()

    config_file = os.environ.get("ROBOT_NAME", "default") + ".yaml"
    config_path = bl.find("bitbots_extrinsic_calibration", config_file, "config")

    bl.node(
        "bitbots_extrinsic_calibration",
        "extrinsic_calibration",
        "bitbots_extrinsic_camera_calibration",
        use_sim_time=sim,
        param_files=config_path,
        params={
            "parent_frame": "camera_optical_frame_left_uncalibrated",
            "child_frame": "zed_left_camera_optical_frame",
        },
    )
    bl.node(
        "bitbots_extrinsic_calibration",
        "extrinsic_calibration",
        "bitbots_extrinsic_imu_calibration",
        use_sim_time=sim,
        param_files=config_path,
        params={
            "parent_frame": "imu_frame_uncalibrated",
            "child_frame": "imu_frame",
        },
    )
