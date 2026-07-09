#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def simulator_teamplayer(
    audio: bool = False,
    behavior: bool = True,
    behavior_dsd_file: str = "main.dsd",
    game_controller: bool = True,
    ipm: bool = True,
    localization: bool = True,
    motion: bool = True,
    path_planning: bool = True,
    teamcom: bool = False,
    vision: bool = True,
    world_model: bool = True,
    rl_motion: bool = True,
    web: bool = False,
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
    teamcom : bool
        Whether the team communication system should be started
    vision : bool
        Whether the vision system should be started
    world_model : bool
        Whether the world model should be started
    rl_motion : bool
        Whether to use the RL motion system instead of the regular one
    web : bool
        Use web-based mjviser viewer instead of the native MuJoCo viewer
    """
    bl = BetterLaunch(pass_launch_func_default=False)

    # load the general simulator
    bl.include("bitbots_mujoco_sim", "simulator.launch.py", web=web)

    # load teamplayer software stack
    bl.include(
        "bitbots_bringup",
        "teamplayer.launch.py",
        audio=audio,
        behavior=behavior,
        behavior_dsd_file=behavior_dsd_file,
        game_controller=game_controller,
        ipm=ipm,
        localization=localization,
        motion=motion,
        sim=True,
        path_planning=path_planning,
        teamcom=teamcom,
        vision=vision,
        world_model=world_model,
        rl_motion=rl_motion,
    )
