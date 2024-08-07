#!/usr/bin/python3
import json
import os

import rclpy
from bitbots_tts.tts import speak
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from bitbots_msgs.msg import Audio


class WorkspaceStatusPublisher(Node):
    def __init__(self):
        super().__init__("WorkspaceStatusPublisher")

        # Declare and get parameters
        self.declare_parameter("workspace_status_path", rclpy.Parameter.Type.STRING)
        self.declare_parameter("publish_topic", "/workspace_status")
        self.workspace_status_path: str = self.get_parameter("workspace_status_path").value
        self.publish_topic: str = self.get_parameter("publish_topic").value

        if not os.path.exists(self.workspace_status_path):
            self.get_logger().warning(
                f"The workspace status file does not exist under the given path: '{self.workspace_status_path}'. Exiting."
            )
            self.destroy_node()
            import sys

            sys.exit(1)

        # Read workspace status file
        self.get_logger().debug(f"Reading workspace status from '{self.workspace_status_path}'...")
        with open(self.workspace_status_path) as f:
            self.workspace_status = f.read()

        # Create publisher
        self.get_logger().debug(f"Creating publisher at '{self.publish_topic}'...")
        self.pub = self.create_publisher(
            String, self.publish_topic, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.get_logger().info("Creating speak publisher...")
        self.speak_pub = self.create_publisher(Audio, "/speak", 1)

        # Publish workspace status
        self.get_logger().debug("Publishing workspace status...")
        self.pub.publish(String(data=self.workspace_status))
        self.get_logger().info("Published workspace status.")

        # Speak workspace status
        self.get_logger().debug("Speaking workspace status...")
        parsed_workspace_status = json.loads(self.workspace_status)
        speak(f"Using software version: ~ {parsed_workspace_status['__WORKSPACE__']['name']}!", self.speak_pub, 30)
        self.get_logger().info("Spoke workspace status.")

        # NOTE: We cannot destroy the node here, because we want to keep the publisher alive.


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()
