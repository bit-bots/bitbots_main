#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl


def normalize_quaternion(quaternion_msg):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm ** (-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


class IMUMarker(Node):
    def __init__(self):
        super().__init__("imu_marker")
        self.marker_name = "IMU"

        # Publisher for the IMU data message (our output)
        self.imu_publisher = self.create_publisher(Imu, "/imu/data", 1)
        # Create the interactive marker server that will handle the IMU marker in RVIZ
        self.server = InteractiveMarkerServer(self, "interactive_imu")

        # Initialize the marker, so that the quaternion is valid
        self.pose = Pose()
        self.pose.orientation.w = 1.0

        # Create the interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "odom"
        int_marker.pose = self.pose
        int_marker.scale = 1.0
        int_marker.name = self.marker_name
        int_marker.description = "Rotate 2DOF to simulate IMU orientation to ground"

        # Create a control that allows to rotate the marker around the x-axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        normalize_quaternion(control.orientation)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.always_visible = True
        int_marker.controls.append(control)

        # Create a control that allows to rotate the marker around the y-axis
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        normalize_quaternion(control.orientation)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Submit the marker to the server, so it can be displayed in RVIZ and we can receive feedback
        self.server.insert(int_marker, feedback_callback=self.process_feedback)

        # Tell the server to apply the changes
        self.server.applyChanges()

        # Create a timer that will publish the IMU data at a fixed rate
        self.timer = self.create_timer(0.05, self.publish_imu_data)

    def process_feedback(self, feedback):
        self.pose = feedback.pose

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation = self.pose.orientation

        self.imu_publisher.publish(imu_msg)


if __name__ == "__main__":
    rclpy.init(args=None)
    marker = IMUMarker()
    # Run the executor (this will block until the node is shutdown)
    rclpy.spin(marker)
