#!/usr/bin/env python3

import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

"""
This script simulates precision messages coming from a perfect localization.
"""


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("rviz_localization_sim")

    pose_publisher = node.create_publisher(PoseWithCovarianceStamped, "pose_with_covariance", 10)

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"

    if len(sys.argv) > 1 and sys.argv[1] == "--bad":
        pose.pose.covariance[0] = 100
        pose.pose.covariance[7] = 100
        pose.pose.covariance[35] = 100

    def timer_callback():
        pose.header.stamp = node.get_clock().now().to_msg()
        pose_publisher.publish(pose)

    timer = node.create_timer(0.05, timer_callback)  # 20 Hz

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
