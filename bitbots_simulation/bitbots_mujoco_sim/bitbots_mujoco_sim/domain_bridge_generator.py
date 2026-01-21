import os
from pathlib import Path

import yaml


class DomainBridgeConfigGenerator:
    """Generates ROS 2 domain bridge configuration for all robots."""

    def __init__(self, robots: list):
        self.main_domain = int(os.getenv("ROS_DOMAIN_ID", "0"))
        self.robots = robots

    def generate_config_file(self, output_dir: Path) -> Path:
        """Generate a single config file for all robots with bidirectional bridging."""
        output_dir.mkdir(parents=True, exist_ok=True)

        output_path = output_dir / "multi_robot_bridge.yaml"
        config = self.generate_config()
        with open(output_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)

        return output_path

    def generate_config(self) -> dict:
        """Generate unified configuration for all robots."""
        config = {
            "name": "multi_robot_bridge",
            "from_domain": self.main_domain,
            "to_domain": self.main_domain,  # Default, overridden per-topic
            "topics": {},
        }

        # Clock is shared - add it once for the first robot domain
        # (only bridges to one domain, but multiple robots can subscribe in that domain)
        if self.robots:
            config["topics"]["clock"] = {
                "type": "rosgraph_msgs/msg/Clock",
                "to_domain": self.robots[0].domain,
            }

        for robot in self.robots:
            # Sensor topics: main → robot (with remap to remove namespace)
            sensor_topics = [
                ("joint_states", "sensor_msgs/msg/JointState"),
                ("imu/data", "sensor_msgs/msg/Imu"),
                ("camera/image_proc", "sensor_msgs/msg/Image"),
                ("camera/camera_info", "sensor_msgs/msg/CameraInfo"),
                ("foot_pressure_left/raw", "bitbots_msgs/msg/FootPressure"),
                ("foot_pressure_right/raw", "bitbots_msgs/msg/FootPressure"),
                ("foot_center_of_pressure_left", "geometry_msgs/msg/PointStamped"),
                ("foot_center_of_pressure_right", "geometry_msgs/msg/PointStamped"),
            ]

            for topic_suffix, msg_type in sensor_topics:
                # YAML key is the SOURCE topic (what we subscribe to in main domain)
                src_topic = f"{robot.namespace}/{topic_suffix}"
                config["topics"][src_topic] = {
                    "type": msg_type,
                    "to_domain": robot.domain,
                    "remap": topic_suffix,  # Remove namespace in robot domain
                }

            # Command topic: robot → main
            # Subscribe to "DynamixelController/command" in robot domain, publish to "robot{N}/DynamixelController/command" in main
            config["topics"][f"DynamixelController/command_{robot.namespace}"] = {
                "type": "bitbots_msgs/msg/JointCommand",
                "from_domain": robot.domain,
                "to_domain": self.main_domain,
                "remap": f"{robot.namespace}/DynamixelController/command",
            }

        return config
