"""Launch the referee separately from the simulator and robot processes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from bitbots_auto_referee.config import PARAMETERS


def generate_launch_description():
    arguments = []
    parameters = {}
    for name, spec in PARAMETERS.items():
        default = str(spec.default).lower() if isinstance(spec.default, bool) else str(spec.default)
        arguments.append(
            DeclareLaunchArgument(
                name,
                default_value=default,
                description=spec.description,
                choices=list(spec.choices) if spec.choices else None,
            )
        )
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=type(spec.default))

    return LaunchDescription(
        [
            *arguments,
            Node(
                package="bitbots_auto_referee",
                executable="auto_referee",
                name="auto_referee",
                namespace="auto_referee",
                parameters=[parameters],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="bitbots_auto_referee",
                executable="auto_referee_ui",
                name="auto_referee_ui",
                namespace="auto_referee",
                condition=IfCondition(LaunchConfiguration("ui_enabled")),
                output="screen",
            ),
        ]
    )
