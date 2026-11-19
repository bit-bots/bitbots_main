from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

# Per-robot game settings that may be provided as launch arguments (e.g. by the
# multi-robot simulation launch) to override the shared ``game_settings.yaml``
# defaults. Each entry maps the launch argument name to the type its value should
# be cast to before it is passed to the parameter blackboard.
GAME_SETTING_OVERRIDES = {
    "bot_id": int,
    "team_id": int,
    "team_color": int,
    "role": str,
    "position_number": int,
}


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

        # Per-robot game settings can be injected as launch arguments. They are
        # applied after ``game_settings.yaml`` so that explicitly provided values
        # win over the shared defaults. Unset arguments (empty string) are ignored
        # and keep the shared default.
        overrides = {}
        for arg_name, cast in GAME_SETTING_OVERRIDES.items():
            value = LaunchConfiguration(arg_name).perform(context)
            if value != "":
                overrides[arg_name] = cast(value)
        if overrides:
            parameters.append(overrides)

        return [
            Node(
                package="demo_nodes_cpp",
                executable="parameter_blackboard",
                name="parameter_blackboard",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=parameters,
                namespace="/",
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
            *[
                DeclareLaunchArgument(
                    arg_name,
                    default_value="",
                    description=(
                        f"Per-robot override for the '{arg_name}' game setting. "
                        "Leave empty to use the shared game_settings.yaml default."
                    ),
                )
                for arg_name in GAME_SETTING_OVERRIDES
            ],
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([get_package_share_directory("bitbots_utils"), "launch", "welcome.launch"])
                )
            ),
            OpaqueFunction(function=create_node),
        ]
    )
