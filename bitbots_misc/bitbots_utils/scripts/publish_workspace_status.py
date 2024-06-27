#!/usr/bin/python3
import os

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


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
                f"The workspace status file does not exist under the given path: '{self.workspace_status_path}'"
            )
            self.destroy_node()

        # Read workspace status file
        with open(self.workspace_status_path) as f:
            self.workspace_status = f.read()

        self.pub = self.create_publisher(
            String, self.publish_topic, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.pub.publish(String(data=self.workspace_status))


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceStatusPublisher()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()
