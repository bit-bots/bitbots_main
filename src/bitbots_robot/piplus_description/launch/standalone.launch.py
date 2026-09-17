#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this
from better_launch.convenience import joint_state_publisher


@launch_this
def standalone(js_pub: bool = True):
    """
    Parameters
    ----------
    js_pub : bool
        Whether to run the joint state publisher
    """
    bl = BetterLaunch()

    bl.include("piplus_description", "rviz.launch.py")
    bl.include("bitbots_parameter_blackboard", "parameter_blackboard.launch.py")
    bl.include("bitbots_robot_description", "load_robot_description.launch.py")

    # We do not have a robot connected, so publish fake joint states
    if js_pub:
        joint_state_publisher(
            use_gui=True,
            node_name="joint_state_publisher_gui",
            anonymous=False,
            params={"rate": 100},
        )
        bl.node("bitbots_utils", "dummy_imu.py", "dummy_imu")
