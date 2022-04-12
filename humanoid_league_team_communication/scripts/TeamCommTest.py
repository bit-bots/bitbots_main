#!/usr/bin/env python3

"""
This script publishes dummy values for ball, goalpost, position and obstacles for testing the team communication.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovariance
from humanoid_league_msgs.msg import ObstacleRelativeArray, ObstacleRelative, PoseWithCertaintyArray, PoseWithCertainty


if __name__ == '__main__':
    rclpy.init(args=None)
    ball_pub = self.create_publisher(PoseWithCertaintyArray, "balls_relative", 1)
    position_pub = self.create_publisher(PoseWithCertainty, "pose_with_certainty", 1)
    obstacle_pub = self.create_publisher(ObstacleRelativeArray, "obstacles_relative", 1)
    position_msg = PoseWithCertainty()
    position_msg.pose.pose.position.x = 2
    position_msg.confidence = 0.7
    obstacle_msg = ObstacleRelativeArray()
    obstacle = ObstacleRelative()
    obstacle.pose.pose.pose.position.x = 4
    obstacle.type = 2
    obstacle_msg.obstacles.append(obstacle)
    obstacle2 = ObstacleRelative()
    obstacle2.pose.pose.pose.position.x = 2
    obstacle2.type = 2
    obstacle_msg.obstacles.append(obstacle2)
    ball_msg = PoseWithCertaintyArray()
    ball = PoseWithCertainty()
    ball.confidence = 1.0
    ball_position = PoseWithCovariance()
    ball_position.pose.position.x = 3
    ball.pose = ball_position
    ball_msg.poses.append(ball)

    while rclpy.ok():
        obstacle_msg.header.stamp = self.get_clock().now()
        obstacle_msg.header.frame_id = "base_footprint"
        ball_msg.header.stamp = self.get_clock().now()
        ball_msg.header.frame_id = "base_footprint"
        ball_pub.publish(ball_msg)
        position_pub.publish(position_msg)
        obstacle_pub.publish(obstacle_msg)
