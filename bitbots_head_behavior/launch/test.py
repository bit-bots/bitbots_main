
import os
import yaml
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    sim_launch_arg = DeclareLaunchArgument(
        "sim", default_value=TextSubstitution(text="false")
    )
    viz_launch_arg = DeclareLaunchArgument(
        "viz", default_value=TextSubstitution(text="false")
    )
    depends_only_launch_arg = DeclareLaunchArgument(
        "depends_only", default_value=TextSubstitution(text="false")
    )
    tf_prefix_launch_arg = DeclareLaunchArgument(
        "tf_prefix", default_value=TextSubstitution(text="")
    )

    body_config = os.path.join(
      get_package_share_directory('bitbots_body_behavior'),
      'config',
      'body_behavior.yaml'
      )
    head_config = os.path.join(
      get_package_share_directory('bitbots_head_behavior'),
      'config',
      'head_config.yaml'
      ) 

    node = Node(
            package='bitbots_head_behavior',
            namespace='',
            executable='head_node',
            parameters=[
                body_config,
                head_config,
                {
                    "camera_frame": "camera",
                    "base_link_frame": "base_link",
                    "odom_frame": "odom",
                    "map_frame": "map",
                    "ball_frame": "ball",
                    "ball_approach_frame": "ball_approach_frame",
                    "base_footprint_frame": "base_footprint"
                }],
            remappings=[("/head_motor_goals", "/DynamixelController/command")]
            )

    return LaunchDescription([
        sim_launch_arg,
        viz_launch_arg,
        depends_only_launch_arg,
        tf_prefix_launch_arg,
        node,
    ])
