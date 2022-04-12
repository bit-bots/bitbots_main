#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped


"""
This script simulates precision messages coming from a perfect localization.
"""


if __name__ == "__main__":

    rclpy.init(args=None)

    pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'pose_with_covariance', 1, latch=True)
    rate = self.create_rate(20)  # rate of 20 Hz

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    if len(sys.argv) > 1 and sys.argv[1] == '--bad':
        pose.pose.covariance[0] = 100
        pose.pose.covariance[7] = 100
        pose.pose.covariance[35] = 100

    while rclpy.ok():
        pose.header.stamp = self.get_clock().now()
        pose_publisher.publish(pose)
        rate.sleep()
