#!/usr/bin/env python3

import math
import sys

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from transforms3d.euler import quat2euler


class TransformMonitor(Node):
    def __init__(self, parent_frame, child_frame):
        super().__init__("transform_monitor")

        self.parent_frame = parent_frame
        self.child_frame = child_frame

        self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == self.parent_frame and transform.child_frame_id == self.child_frame:
                # Extract position
                pos = transform.transform.translation

                # Extract rotation as quaternion and convert to euler angles (in degrees)
                rot = transform.transform.rotation
                roll, pitch, yaw = quat2euler([rot.w, rot.x, rot.y, rot.z])
                roll_deg = math.degrees(roll)
                pitch_deg = math.degrees(pitch)
                yaw_deg = math.degrees(yaw)

                # Print aligned output
                print(f"Pos:  x: {pos.x:8.4f}  y: {pos.y:8.4f}  z: {pos.z:8.4f}")
                print(f"Rot:  r: {roll_deg:8.2f}  p: {pitch_deg:8.2f}  y: {yaw_deg:8.2f}")
                print()


def main():
    if len(sys.argv) < 3:
        print("Usage: monitor_odometry.py <parent_frame> <child_frame>")
        print("Example: monitor_odometry.py odom base_link")
        sys.exit(1)

    parent_frame = sys.argv[1]
    child_frame = sys.argv[2]

    rclpy.init()
    node = TransformMonitor(parent_frame, child_frame)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
