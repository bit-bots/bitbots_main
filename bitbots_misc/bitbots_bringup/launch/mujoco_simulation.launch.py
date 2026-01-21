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
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_domain_bridge_config(robot_domain: int, output_dir: Path) -> Path:
    """Generate domain bridge config file for a single robot.

    Returns the config file path.
    """
    main_domain = int(os.getenv("ROS_DOMAIN_ID", "0"))
    output_dir.mkdir(parents=True, exist_ok=True)

    namespace = f"robot{robot_domain}"

    config = {
        "name": f"robot{robot_domain}_bridge",
        "from_domain": main_domain,
        "to_domain": robot_domain,
        "topics": {
            # Clock: main → robot domain
            "clock": {
                "type": "rosgraph_msgs/msg/Clock",
            },
        },
    }

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
            "remap": topic_suffix,
        }

    # Command topic: robot domain → main (reversed direction)
    # Key is source topic in from_domain, remap is destination in to_domain
    config["topics"]["DynamixelController/command"] = {
        "type": "bitbots_msgs/msg/JointCommand",
        "from_domain": robot_domain,
        "to_domain": main_domain,
        "remap": f"{namespace}/DynamixelController/command",
    }

    config_path = output_dir / f"robot{robot_domain}_bridge.yaml"
    with open(config_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    return config_path


def generate_world_xml(num_robots: int, package_share: str) -> Path:
    """Generate MuJoCo world XML with the correct number of robots."""
    template_path = Path(package_share) / "xml" / "adult_field.xml"
    output_path = Path(package_share) / "xml" / "generated_world.xml"

    with open(template_path) as f:
        template = f.read()

    # Replace placeholder with actual robot count
    world_xml = template.replace("{{NUM_ROBOTS}}", str(num_robots))

    with open(output_path, "w") as f:
        f.write(world_xml)

    return output_path


def launch_setup(context):
    """Dynamically set up launches based on num_robots."""
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    package_share = get_package_share_directory("bitbots_mujoco_sim")
    bridge_config_dir = Path(package_share) / "config" / "domain_bridges"

    world_file = generate_world_xml(num_robots, package_share)

    actions = []

    actions.append(
        LogInfo(msg=f"Starting MuJoCo simulation with {num_robots} robot(s)"),
    )
    actions.append(
        Node(
            package="bitbots_mujoco_sim",
            executable="sim",
            name="sim_interface",
            output="screen",
            emulate_tty=True,
            parameters=[{"world_file": str(world_file)}],
        ),
    )

    for robot_domain in range(num_robots):
        config_file = generate_domain_bridge_config(robot_domain, bridge_config_dir)
        actions.append(
            LogInfo(msg=f"Starting domain bridge for robot{robot_domain} (domain {robot_domain})"),
        )
        actions.append(
            Node(
                package="domain_bridge",
                executable="domain_bridge",
                name=f"domain_bridge_robot{robot_domain}",
                arguments=[str(config_file)],
                output="screen",
                emulate_tty=True,
            ),
        )

        actions.append(
            TimerAction(
                period=3.0,
                actions=[
                    LogInfo(msg=f"Launching teamplayer stack for robot{robot_domain} in domain {robot_domain}"),
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "launch",
                            "bitbots_bringup",
                            "teamplayer.launch",
                            "sim:=true",
                            "teamcom:=false",  # Disable team_comm to avoid port conflicts
                        ],
                        output="screen",
                        additional_env={"ROS_DOMAIN_ID": str(robot_domain)},
                    ),
                ],
            )
        )

    return actions


def generate_launch_description():
    """Launch MuJoCo simulation with domain bridge for multi-robot support."""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_robots",
                default_value="1",
                description="Number of robots in the simulation",
            ),
            # All setup happens in OpaqueFunction to ensure proper ordering
            OpaqueFunction(function=launch_setup),
        ]
    )
