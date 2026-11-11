from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    def create_node(context, *args, **kwargs):
        in_sim = LaunchConfiguration("sim").perform(context) == "true"
        field_name = LaunchConfiguration("fieldname").perform(context)
        parameters_file = LaunchConfiguration("parameters_file").perform(context)

        parameters: list = [
            {"simulation_active": in_sim},
            {"use_sim_time": in_sim},
            {"field.name": field_name},
        ]
        if parameters_file:
            # Later entries take precedence, so the overrides win over the settings above
            parameters.append(parameters_file)

        # The node loads the configuration files (field, global parameters, game settings) itself
        return [
            Node(
                package="bitbots_parameter_blackboard",
                executable="parameter_blackboard",
                name="parameter_blackboard",
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
            DeclareLaunchArgument(
                "parameters_file",
                default_value="",
                description="ROS parameter file with overrides of global parameters, e.g. passed by "
                "bitbots_compose_server. They take precedence over the configuration files and may only "
                "replace parameters defined there.",
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([get_package_share_directory("bitbots_utils"), "launch", "welcome.launch"])
                )
            ),
            OpaqueFunction(function=create_node),
        ]
    )
