"""Generates domain bridge configuration files for multi-robot simulation."""

from pathlib import Path

import yaml


class DomainBridgeConfigGenerator:
    """Generates ROS 2 domain bridge configuration for isolating multiple robots."""

    def __init__(self, robot_count: int, main_domain: int = 0):
        """
        Initialize the domain bridge config generator.

        Args:
            robot_count: Number of robots in the simulation
            main_domain: The main domain ID (default: 0)
        """
        self.robot_count = robot_count
        self.main_domain = main_domain

    def generate_config(self, robot_index: int) -> dict:
        """
        Generate domain bridge config for a specific robot.

        Args:
            robot_index: The robot index (0-based)

        Returns:
            Dictionary containing the bridge configuration
        """
        robot_domain = self.main_domain + robot_index + 1
        robot_namespace = f"robot{robot_index}"

        config = {
            "name": f"robot_{robot_index}_bridge",
            "from_domain": self.main_domain,
            "to_domain": robot_domain,
            "topics": {},
        }

        # Topics from main domain to robot domain (robot subscribes)
        robot_topics = {
            f"{robot_namespace}/DynamixelController/command": {"type": "bitbots_msgs/msg/JointCommand", "dir": "TX"}
        }

        # Topics from robot domain to main domain (robot publishes)
        main_topics = {
            "clock": {"type": "rosgraph_msgs/msg/Clock", "dir": "RX"},
            f"{robot_namespace}/joint_states": {"type": "sensor_msgs/msg/JointState", "dir": "RX"},
            f"{robot_namespace}/imu/data_raw": {"type": "sensor_msgs/msg/Imu", "dir": "RX"},
            f"{robot_namespace}/camera/image_proc": {"type": "sensor_msgs/msg/Image", "dir": "RX"},
            f"{robot_namespace}/camera/camera_info": {"type": "sensor_msgs/msg/CameraInfo", "dir": "RX"},
            f"{robot_namespace}/foot_pressure_left/filtered": {"type": "bitbots_msgs/msg/FootPressure", "dir": "RX"},
            f"{robot_namespace}/foot_pressure_right/filtered": {"type": "bitbots_msgs/msg/FootPressure", "dir": "RX"},
            f"{robot_namespace}/foot_center_of_pressure_left": {"type": "geometry_msgs/msg/PointStamped", "dir": "RX"},
            f"{robot_namespace}/foot_center_of_pressure_right": {"type": "geometry_msgs/msg/PointStamped", "dir": "RX"},
        }

        # Merge topics
        config["topics"].update(robot_topics)
        config["topics"].update(main_topics)

        return config

    def save_config(self, robot_index: int, output_path: Path) -> None:
        """
        Save domain bridge config to YAML file.

        Args:
            robot_index: The robot index
            output_path: Path where to save the config file
        """
        config = self.generate_config(robot_index)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    def generate_all_configs(self, output_dir: Path) -> list[Path]:
        """
        Generate config files for all robots.

        Args:
            output_dir: Directory where to save all config files

        Returns:
            List of paths to generated config files
        """
        output_dir.mkdir(parents=True, exist_ok=True)
        config_paths = []

        for i in range(self.robot_count):
            config_path = output_dir / f"robot_{i}_bridge.yaml"
            self.save_config(i, config_path)
            config_paths.append(config_path)

        return config_paths
