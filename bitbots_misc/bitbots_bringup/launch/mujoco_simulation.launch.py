import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_domain_bridge_config(num_robots: int, output_path: Path) -> None:
    """Generate domain bridge config file for multi-robot simulation."""
    main_domain = int(os.getenv("ROS_DOMAIN_ID", "0"))

    config = {
        "name": "multi_robot_bridge",
        "from_domain": main_domain,
        "to_domain": main_domain,
        "topics": {},
    }

    # Clock is shared - bridge to first robot domain
    if num_robots > 0:
        config["topics"]["clock"] = {
            "type": "rosgraph_msgs/msg/Clock",
            "to_domain": 0,
        }

    for robot_id in range(num_robots):
        namespace = f"robot{robot_id}"
        domain = robot_id

        # Sensor topics: main → robot domain (with remap to remove namespace)
        sensor_topics = [
            ("joint_states", "sensor_msgs/msg/JointState"),
            ("imu/data", "sensor_msgs/msg/Imu"),
            ("camera/image_proc", "sensor_msgs/msg/Image"),
            ("camera/camera_info", "sensor_msgs/msg/CameraInfo"),
            ("foot_pressure_left/filtered", "bitbots_msgs/msg/FootPressure"),
            ("foot_pressure_right/filtered", "bitbots_msgs/msg/FootPressure"),
            ("foot_center_of_pressure_left", "geometry_msgs/msg/PointStamped"),
            ("foot_center_of_pressure_right", "geometry_msgs/msg/PointStamped"),
        ]

        for topic_suffix, msg_type in sensor_topics:
            src_topic = f"{namespace}/{topic_suffix}"
            config["topics"][src_topic] = {
                "type": msg_type,
                "to_domain": domain,
                "remap": topic_suffix,
            }

        # Command topic: robot domain → main
        config["topics"][f"DynamixelController/command_{namespace}"] = {
            "type": "bitbots_msgs/msg/JointCommand",
            "from_domain": domain,
            "to_domain": main_domain,
            "remap": f"{namespace}/DynamixelController/command",
        }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)


def launch_setup(context):
    """Dynamically set up launches based on num_robots."""
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    package_share = get_package_share_directory("bitbots_mujoco_sim")
    bridge_config = Path(package_share) / "config" / "domain_bridges" / "multi_robot_bridge.yaml"

    # Generate domain bridge config
    generate_domain_bridge_config(num_robots, bridge_config)

    actions = []

    # Start domain bridge (config is already generated, no delay needed)
    actions.append(
        LogInfo(msg=f"Starting domain bridge with config: {bridge_config}"),
    )
    actions.append(
        Node(
            package="domain_bridge",
            executable="domain_bridge",
            name="domain_bridge",
            arguments=[str(bridge_config)],
            output="screen",
            emulate_tty=True,
        ),
    )

    # Launch a teamplayer stack for each robot in its own domain (0-indexed to match simulation)
    for robot_id in range(num_robots):
        actions.append(
            LogInfo(msg=f"Launching teamplayer stack for robot{robot_id} in domain {robot_id}"),
        )
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "bitbots_bringup",
                    "teamplayer.launch",
                    "sim:=true",
                ],
                output="screen",
                additional_env={"ROS_DOMAIN_ID": str(robot_id)},
            ),
        )

    return actions


def generate_launch_description():
    """Launch MuJoCo simulation with domain bridge for multi-robot support."""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_robots",
                default_value="1",
                description="Number of robots in the simulation (must match MuJoCo world)",
            ),
            Node(
                package="bitbots_mujoco_sim",
                executable="sim",
                name="sim_interface",
                output="screen",
                emulate_tty=True,
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
