from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    package_name = "bitbots_parameter_blackboard"
    package_share = get_package_share_directory(package_name)

    sim = LaunchConfiguration("sim")
    field_name = PythonExpression(["'hsl_kid' if '", sim, "' == 'true' else 'labor'"])

    parameters = [
        {"simulation_active": sim},
        {"use_sim_time": sim},
        {"field.name": field_name},
        ParameterFile(PathJoinSubstitution([package_share, "config", "fields", field_name, "config.yaml"])),
        ParameterFile(PathJoinSubstitution([package_share, "config", "global_parameters.yaml"])),
    ]

    robot_domain = os.environ.get("ROS_DOMAIN_ID")
    if robot_domain is not None:
        parameters.append(
            ParameterFile(
                PathJoinSubstitution([package_share, "config", f"sim_game_settings_{int(robot_domain)}.yaml"])
            )
        )
    else:
        parameters.append(ParameterFile(PathJoinSubstitution([package_share, "config", "game_settings.yaml"])))

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="false"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([get_package_share_directory("bitbots_utils"), "launch", "welcome.launch"])
                )
            ),
            Node(
                package="demo_nodes_cpp",
                executable="parameter_blackboard",
                name="parameter_blackboard",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=parameters,
            ),
        ]
    )
