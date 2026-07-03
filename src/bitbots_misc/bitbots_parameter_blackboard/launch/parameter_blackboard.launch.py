from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    package_name = "bitbots_parameter_blackboard"
    package_share = get_package_share_directory(package_name)

    def create_node(context, *args, **kwargs):
        in_sim = LaunchConfiguration("sim").perform(context) == "true"
        field_name = LaunchConfiguration("fieldname").perform(context)

        parameters = [
            {"simulation_active": in_sim},
            {"use_sim_time": in_sim},
            {"field.name": field_name},
            ParameterFile(PathJoinSubstitution([package_share, "config", "fields", field_name, "config.yaml"])),
            ParameterFile(PathJoinSubstitution([package_share, "config", "global_parameters.yaml"])),
            ParameterFile(PathJoinSubstitution([package_share, "config", "game_settings.yaml"])),
        ]

        robot_domain = os.environ.get("ROS_DOMAIN_ID")
        if in_sim and robot_domain is not None:
            parameters.append(
                ParameterFile(
                    PathJoinSubstitution([package_share, "config", f"sim_game_settings_{int(robot_domain)}.yaml"])
                )
            )

        return [
            Node(
                package="demo_nodes_cpp",
                executable="parameter_blackboard",
                name="parameter_blackboard",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=parameters,
            )
        ]

    sim = LaunchConfiguration("sim")
    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="false"),
            DeclareLaunchArgument(
                "fieldname",
                default_value=PythonExpression(["'hsl_kid' if '", sim, "' == 'true' else 'labor'"]),
                description="Field name to load parameters for.",
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([get_package_share_directory("bitbots_utils"), "launch", "welcome.launch"])
                )
            ),
            OpaqueFunction(function=create_node),
        ]
    )
