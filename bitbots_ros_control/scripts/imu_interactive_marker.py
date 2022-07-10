#!/usr/bin/env python3

import traceback

import rclpy
from rclpy.node import Node
import copy
import math

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import Imu

rclpy.init(args=None)


def normalize_quaternion(quaternion_msg):
    norm = quaternion_msg.x ** 2 + quaternion_msg.y ** 2 + quaternion_msg.z ** 2 + quaternion_msg.w ** 2
    s = norm ** (-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


class IMUMarker:

    def __init__(self, server):
        self.marker_name = "IMU"
        self.imu_publisher = self.create_publisher(Imu, "/imu/data", 1)
        self.server = server
        self.pose = Pose()
        self.pose.orientation.w = 1

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "odom"
        int_marker.pose = self.pose
        int_marker.scale = 1
        int_marker.name = self.marker_name
        int_marker.description = "Rotate 2DOF to simulate IMU orientation to ground"

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalize_quaternion(control.orientation)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.always_visible = True
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalize_quaternion(control.orientation)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # we want to use our special callback function
        self.server.insert(int_marker, self.process_feedback)

    def process_feedback(self, feedback):
        self.pose = feedback.pose

    def publish_marker(self, e):
        # create IMU message and publish
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.get_rostime()
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation = self.pose.orientation

        self.imu_publisher.publish(imu_msg)


if __name__ == "__main__":
    server = InteractiveMarkerServer("interactive_imu")
    imu = IMUMarker(server)

    server.applyChanges()

    # create a timer to update the published imu transform
    rospy.Timer(Duration(seconds=0.05), imu.publish_marker)

    # run and block until finished
    rclpy.spin(self)
