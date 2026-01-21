from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Launch MuJoCo simulation with domain bridge for multi-robot support."""

    package_share = get_package_share_directory("bitbots_mujoco_sim")
    bridge_config = Path(package_share) / "config" / "domain_bridges" / "multi_robot_bridge.yaml"
    bringup_share = get_package_share_directory("bitbots_bringup")

    # Motion standalone arguments
    motion_arg = DeclareLaunchArgument("motion", default_value="false", description="Launch motion standalone stack")

    return LaunchDescription(
        [
            motion_arg,
            Node(
                package="bitbots_mujoco_sim",
                executable="sim",
                name="sim_interface",
                output="screen",
                emulate_tty=True,
            ),
            # Motion stack (delayed 5s, only when motion is enabled)
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            PathJoinSubstitution([bringup_share, "launch", "motion_standalone.launch"])
                        ),
                        launch_arguments={"sim": "true"}.items(),
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("motion")),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    LogInfo(msg=f"Starting domain bridge with config: {bridge_config}"),
                    Node(
                        package="domain_bridge",
                        executable="domain_bridge",
                        name="domain_bridge",
                        arguments=[str(bridge_config)],
                        output="screen",
                        emulate_tty=True,
                    ),
                ],
            ),
        ]
    )
