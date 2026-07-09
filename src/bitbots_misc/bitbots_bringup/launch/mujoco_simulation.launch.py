#!/usr/bin/env python3
import os
from pathlib import Path

import yaml

from better_launch import BetterLaunch, launch_this

# Teamplayer arguments to expose (name, description)
# Empty string default means "not set" - will use teamplayer's default
# sim is always true for mujoco simulation
TEAMPLAYER_ARGS = [
    ("audio", "Whether the audio system should be started"),
    ("behavior", "Whether the behavior control system should be started"),
    ("behavior_dsd_file", "The behavior dsd file that should be used"),
    ("game_controller", "Whether the Gamecontroller module should be started"),
    ("ipm", "Whether the inverse perspective mapping should be started"),
    ("localization", "Whether the localization system should be started"),
    ("motion", "Whether the motion control system should be started"),
    ("path_planning", "Whether the path planning should be started"),
    ("teamcom", "Whether the team communication system should be started"),
    ("vision", "Whether the vision system should be started"),
    ("world_model", "Whether the world model should be started"),
    ("monitoring", "Whether the system monitor and udp bridge should be started"),
    ("record", "Whether the ros bag recording should be started"),
    ("tts", "Whether to speak"),
]


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
        ("zed/zed_node/rgb/image_rect_color", "sensor_msgs/msg/Image"),
        ("zed/zed_node/rgb/camera_info", "sensor_msgs/msg/CameraInfo"),
    ]

    for topic_suffix, msg_type in sensor_topics:
        src_topic = f"{namespace}/{topic_suffix}"
        config["topics"][src_topic] = {
            "type": msg_type,
            "remap": topic_suffix,
        }

    # Command topic: robot domain → main (reversed direction)
    # Key is source topic in from_domain, remap is destination in to_domain
    config["topics"]["joint_command"] = {
        "type": "bitbots_msgs/msg/JointCommand",
        "from_domain": robot_domain,
        "to_domain": main_domain,
        "remap": f"{namespace}/joint_command",
    }

    config_path = output_dir / f"robot{robot_domain}_bridge.yaml"
    with open(config_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    return config_path


def generate_world_xml(num_robots: int, package_share: str, robot_type: str) -> Path:
    """Generate MuJoCo world XML with the correct number of robots."""
    template_path = Path(package_share) / "xml" / "kid_field.xml"
    output_path = Path(package_share) / "xml" / "generated_world.xml"
    offset = 4 * (
        1 / num_robots
    )  # this makes the offset be the default value when there are 4 robots and increse the less robots there are

    with open(template_path) as f:
        template = f.read()

    # Replace placeholder with actual robot count
    world_xml = (
        template.replace("{{NUM_ROBOTS}}", str(num_robots))
        .replace("{{OFFSET}}", str(offset))
        .replace("{{ROBOT_TYPE}}", robot_type)
    )

    with open(output_path, "w") as f:
        f.write(world_xml)
    return output_path


@launch_this
def mujoco_simulation(
    num_robots: int = 1,
    robot_type: str = "piplus",
    audio: str = "",
    behavior: str = "",
    behavior_dsd_file: str = "",
    game_controller: str = "",
    ipm: str = "",
    localization: str = "",
    motion: str = "",
    path_planning: str = "",
    teamcom: str = "",
    vision: str = "",
    world_model: str = "",
    monitoring: str = "",
    record: str = "",
    tts: str = "",
):
    """Launch MuJoCo simulation with domain bridge for multi-robot support.

    Parameters
    ----------
    num_robots : int
        Number of robots in the simulation
    robot_type : str
        Set the type of robot used (piplus, x02)
    """
    bl = BetterLaunch()

    # Only forwarded to the per-robot teamplayer stack if explicitly set (not empty)
    teamplayer_values = {
        "audio": audio,
        "behavior": behavior,
        "behavior_dsd_file": behavior_dsd_file,
        "game_controller": game_controller,
        "ipm": ipm,
        "localization": localization,
        "motion": motion,
        "path_planning": path_planning,
        "teamcom": teamcom,
        "vision": vision,
        "world_model": world_model,
        "monitoring": monitoring,
        "record": record,
        "tts": tts,
    }

    package_share = Path(bl.find("bitbots_mujoco_sim")) / "share" / "bitbots_mujoco_sim"
    bridge_config_dir = Path(package_share) / "config" / "domain_bridges"

    teamplayer_args = ["--sim", "true"]  # sim is always true for mujoco simulation
    for arg_name, value in teamplayer_values.items():
        if value:  # Only pass if not empty string
            teamplayer_args += [f"--{arg_name}", value]

    world_file = generate_world_xml(num_robots, package_share, robot_type)

    bl.logger.info(f"Starting MuJoCo simulation with {num_robots} robot(s)")
    bl.node(
        "bitbots_mujoco_sim",
        "sim",
        "sim_interface",
        params={"world_file": str(world_file)},
    )

    for robot_domain in range(11, num_robots + 11):  # 11 is the standard starting id for our robots
        config_file = generate_domain_bridge_config(robot_domain, bridge_config_dir)

        bl.logger.info(f"Starting domain bridge for robot{robot_domain} (domain {robot_domain})")
        bl.node(
            "domain_bridge",
            "domain_bridge",
            f"domain_bridge_robot{robot_domain}",
            cmd_args=[str(config_file)],
        )

        def start_teamplayer(robot_domain=robot_domain):
            bl.logger.info(f"Launching teamplayer stack for robot{robot_domain} in domain {robot_domain}")
            bl.process(
                ["bl", "bitbots_bringup", "teamplayer.launch.py"] + teamplayer_args,
                name=f"teamplayer_robot{robot_domain}",
                output="screen",
                env={"ROS_DOMAIN_ID": str(robot_domain)},
            )

        bl.run_later(3.0, start_teamplayer)
