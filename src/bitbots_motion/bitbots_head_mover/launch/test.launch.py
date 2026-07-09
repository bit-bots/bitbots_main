#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def test(sim: bool = False, viz: bool = False, tf_prefix: str = ""):
    bl = BetterLaunch()

    if not sim and not viz:
        bl.include("livelybot_bringup", "lowlevel.launch")

    bl.include("bitbots_parameter_blackboard", "parameter_blackboard.launch.py", sim=sim)
    bl.include("bitbots_robot_description", "load_robot_description.launch.py", sim=sim)

    # launch the base footprint
    bl.node(
        "humanoid_base_footprint",
        "base_footprint",
        "base_footprint",
        params={"support_state_topics": ["walk_support_state"]},
        use_sim_time=sim,
    )

    # launch the odometry
    bl.include("bitbots_odometry", "odometry.launch.py", sim=sim)

    bl.include("bitbots_ball_filter", "ball_filter.launch.py", sim=sim)

    if viz:
        # translate joint goals to joint states
        bl.node("bitbots_utils", "motor_goals_viz_helper.py", "MotorGoalsVizHelper", cmd_args=["--head"])
        # fake IMU needed for odometry
        bl.node("bitbots_utils", "dummy_imu.py", "DummyImu")
        # create fake tf from map to robot
        bl.node(
            "tf2_ros",
            "static_transform_publisher",
            "static_map2odom_tf",
            cmd_args=[
                "--x", "-0.0", "--y", "-0.0", "--z", "0.0",
                "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
            log_level=None,
        )
    else:
        # launch vision
        bl.include("bitbots_bringup", "vision.launch.py", sim=sim)

        # launch inverse perspective mapping (ipm)
        bl.include("bitbots_ipm", "ipm.launch.py", sim=sim)

    bl.include("bitbots_head_mover", "head_mover.launch.py", sim=sim)
