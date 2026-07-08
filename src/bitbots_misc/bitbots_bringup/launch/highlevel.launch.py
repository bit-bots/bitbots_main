#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def highlevel(
    audio: bool = True,
    behavior_dsd_file: str = "main.dsd",
    behavior: bool = True,
    game_controller: bool = True,
    ipm: bool = True,
    localization: bool = True,
    path_planning: bool = True,
    sim: bool = False,
    teamcom: bool = False,
    vision: bool = True,
    workspace_status: bool = True,
    world_model: bool = True,
    whistle_detector: bool = True,
):
    """
    Parameters
    ----------
    audio : bool
        Whether the audio system should be started
    behavior_dsd_file : str
        The behavior dsd file that should be used
    behavior : bool
        Whether the behavior control system should be started
    game_controller : bool
        Whether the Gamecontroller module should be started
    ipm : bool
        Whether the inverse perspective mapping should be started
    localization : bool
        Whether the localization system should be started
    path_planning : bool
        Whether the path planning should be started
    sim : bool
        Whether the robot is running in simulation or on real hardware
    teamcom : bool
        Whether the team communication system should be started
    vision : bool
        Whether the vision system should be started
    workspace_status : bool
        Whether to publish the current workspace status
    world_model : bool
        Whether the world model should be started
    whistle_detector : bool
        Whether whistle detector should be started
    """
    bl = BetterLaunch()

    # launch game controller
    if game_controller:
        bl.include(
            "game_controller_hsl",
            "game_controller.launch",
            sim=sim,
            use_parameter_blackboard=True,
            parameter_blackboard_name="parameter_blackboard",
            team_id_param_name="team_id",
            bot_id_param_name="bot_id",
        )
        bl.include("bitbots_player_state", "player_state_aggregator.launch.py", sim=sim)

    # launch vision
    if vision:
        bl.include("bitbots_bringup", "vision.launch.py", sim=sim)

    # launch inverse perspective mapping (ipm)
    if ipm:
        bl.include("bitbots_ipm", "ipm.launch.py", sim=sim)

    # launch teamcom
    if teamcom:
        bl.include("bitbots_team_communication", "team_comm.launch.py", sim=sim)

    # launch world model
    if world_model:
        bl.include("bitbots_ball_filter", "ball_filter.launch.py", sim=sim)
        bl.include("bitbots_robot_filter", "robot_filter.launch.py", sim=sim)

    if whistle_detector:
        bl.include("bitbots_whistle_detector", "whistle_detector.launch.py")

    # launch localization or fake localization
    if localization:
        bl.include("bitbots_localization", "localization.launch.py", sim=sim)
    else:
        # simulate map frame
        bl.node(
            "tf2_ros",
            "static_transform_publisher",
            "map_odom",
            cmd_args=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
            log_level=None,
        )
        # publish perfect covariance
        # bl.node("bitbots_localization", "rviz_localization_sim.py", "localization_covariance")

    # launch path planning
    if path_planning:
        bl.include("bitbots_path_planning", "path_planning.launch.py", sim=sim)

    # launch behavior
    if behavior:
        bl.include(
            "bitbots_body_behavior",
            "behavior.launch.py",
            dsd_file=behavior_dsd_file,
            sim=sim,
        )

    # launch audio processing
    if audio:
        bl.include("bitbots_bringup", "audio.launch.py")

    # launch workspace status publisher
    if workspace_status and not sim:
        bl.node(
            "bitbots_utils",
            "publish_workspace_status.py",
            "",
            params={
                "workspace_status_path": bl.find("bitbots_utils", "workspace_status.json", "config"),
                "publish_topic": "/workspace_status",
            },
        )
