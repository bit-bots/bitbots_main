#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def visualization(
    behavior: bool = True,
    ipm: bool = False,
    motion: bool = True,
    game_controller: bool = False,
    fieldname: str = "small_division_2026",
):
    """
    Parameters
    ----------
    behavior : bool
        if the behavior should be started
    ipm : bool
        if the soccer ipm should be used
    motion : bool
        if the motion should be started
    game_controller : bool
        if the game controller node should be started
    fieldname : str
        Loads field settings
    """
    bl = BetterLaunch()

    # load the global parameters
    bl.include("bitbots_parameter_blackboard", "parameter_blackboard.launch.py")

    # publish dummy imu
    bl.node("bitbots_utils", "dummy_imu.py", "dummy_imu")

    # launch motion nodes
    if motion:
        bl.include("bitbots_bringup", "motion.launch.py", viz=True)

    # launch highlevel nodes, except vision and ipm (we have fake vision instead)
    bl.include(
        "bitbots_bringup",
        "highlevel.launch.py",
        behavior=behavior,
        localization=False,
        game_controller=game_controller,
        vision=False,
        ipm=False,
    )

    # simulate localization
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

    # translate joint goals to joint states
    bl.node("bitbots_utils", "motor_goals_viz_helper.py", "motor_goals_viz_helper")

    # add some visualization tools
    bl.include("bitbots_team_communication", "team_comm_test_marker.launch.py", rviz=False)
