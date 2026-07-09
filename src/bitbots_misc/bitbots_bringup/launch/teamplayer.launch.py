#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def teamplayer(
    audio: bool = True,
    behavior: bool = True,
    behavior_dsd_file: str = "main.dsd",
    game_controller: bool = True,
    ipm: bool = True,
    localization: bool = True,
    motion: bool = True,
    path_planning: bool = True,
    sim: bool = False,
    teamcom: bool = True,
    vision: bool = True,
    world_model: bool = True,
    monitoring: bool = False,
    record: bool = False,
    tts: bool = False,
    whistle_detector: bool = True,
    fieldname: str = None,
):
    """
    Parameters
    ----------
    audio : bool
        Whether the audio system should be started
    behavior : bool
        Whether the behavior control system should be started
    behavior_dsd_file : str
        The behavior dsd file that should be used
    game_controller : bool
        Whether the Gamecontroller module should be started
    ipm : bool
        Whether the inverse perspective mapping should be started
    localization : bool
        Whether the localization system should be started
    motion : bool
        Whether the motion control system should be started
    path_planning : bool
        Whether the path planning should be started
    sim : bool
        Whether the robot is running in simulation or on real hardware
    teamcom : bool
        Whether the team communication system should be started
    vision : bool
        Whether the vision system should be started
    world_model : bool
        Whether the world model should be started
    monitoring : bool
        Whether the system monitor and udp bridge should be started
    record : bool
        Whether the ros bag recording should be started
    tts : bool
        Whether to speak
    whistle_detector : bool
        Whether to detect whistles
    fieldname : str
        Loads field settings. Defaults to "hsl_kid" in simulation, "small_division_2026" otherwise.
    """
    bl = BetterLaunch(pass_launch_func_default=False)

    # TODO: use better_launch's own use_sim_time mechanism (bl.group(use_sim_time=...) /
    # Settings().use_sim_time / BL_USE_SIM_TIME) instead of manually threading `sim` through
    # every node()/include() call across all our launch files. Before switching, check how
    # that propagates into included *regular ROS2* launch files (game_controller_hsl,
    # zed_wrapper, livelybot_bringup, domain_bridge, udp_bridge, foxglove_bridge,
    # rosbridge_server) - those run via a separate ROS2 LaunchService and don't share our
    # group stack, so they'd likely still need sim/use_sim_time passed explicitly.

    if fieldname is None:
        fieldname = "hsl_kid" if sim else "small_division_2026"

    # load the global parameters
    bl.include("bitbots_parameter_blackboard", "parameter_blackboard.launch.py", sim=sim, fieldname=fieldname)

    # load the text to speech engine
    if tts:
        bl.include("bitbots_tts", "tts.launch.py")

    # load the diagnostic aggregator
    bl.include("bitbots_diagnostic", "aggregator.launch.py")

    # load the robot description
    bl.include("bitbots_robot_description", "load_robot_description.launch.py", sim=sim)

    # load the motion
    if motion:
        bl.include("bitbots_bringup", "motion.launch.py", sim=sim)

    # load the highlevel stuff
    bl.include(
        "bitbots_bringup",
        "highlevel.launch.py",
        audio=audio,
        behavior=behavior,
        behavior_dsd_file=behavior_dsd_file,
        game_controller=game_controller,
        ipm=ipm,
        localization=localization,
        path_planning=path_planning,
        sim=sim,
        teamcom=teamcom,
        vision=vision,
        world_model=world_model,
        whistle_detector=whistle_detector,
    )

    # load monitoring
    if monitoring and not sim:
        bl.include("bitbots_bringup", "monitoring.launch.py")

    # record rosbag
    if record:
        bl.include("bitbots_bringup", "rosbag_record.launch.py", sim=sim)
