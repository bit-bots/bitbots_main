from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_vision(context):
    backend = LaunchConfiguration("vision_backend").perform(context)
    if backend not in ("original", "schnupper"):
        raise ValueError("vision_backend muss original oder schnupper sein")
    config = Path(get_package_share_directory("bitbots_vision")) / "config" / "visionparams.yaml"
    return [
        Node(
            package="bitbots_vision",
            executable="vision" if backend == "original" else "schnupper_vision_node",
            name="bitbots_vision",
            output="screen",
            parameters=[
                str(config),
                {
                    "components.debug_active": ParameterValue(LaunchConfiguration("debug"), value_type=bool),
                    "use_sim_time": ParameterValue(LaunchConfiguration("sim"), value_type=bool),
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vision_backend", default_value="original", choices=["original", "schnupper"]),
            DeclareLaunchArgument("sim", default_value="false"),
            DeclareLaunchArgument("debug", default_value="false"),
            OpaqueFunction(function=launch_vision),
        ]
    )
