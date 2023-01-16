#!/usr/bin/env python3
"""
This script publishes dummy values for ball, goalpost, position and obstacles for testing the team communication.
"""

import rclpy
import numpy
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
from humanoid_league_msgs.msg import GameState, ObstacleRelativeArray, ObstacleRelative, Strategy


def pose_with_covariance(x, y, z=0.0):
    point = Point(x=x, y=y, z=z)
    quat = Quaternion(x=0.2, y=0.3, z=0.5, w=0.8)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=quat))
    pose.covariance = covariance_list()

    return PoseWithCovarianceStamped(pose=pose)


def covariance_list():
    covariance = numpy.empty(36, dtype=numpy.float64)
    covariance[0] = 1.0
    covariance[6] = 2.0
    covariance[30] = 3.0
    covariance[1] = 4.0
    covariance[7] = 5.0
    covariance[31] = 6.0
    covariance[5] = 7.0
    covariance[11] = 8.0
    covariance[35] = 9.0

    return covariance


if __name__ == '__main__':
    rclpy.init(args=None)
    node = rclpy.create_node("test_team_comm")

    gamestate_pub = node.create_publisher(GameState, "gamestate", 1)
    strategy_pub = node.create_publisher(Strategy, "strategy", 1)
    ball_pub = node.create_publisher(PoseWithCovarianceStamped, "ball_position_relative_filtered", 1)
    position_pub = node.create_publisher(PoseWithCovarianceStamped, "pose_with_covariance", 1)
    obstacle_pub = node.create_publisher(ObstacleRelativeArray, "obstacles_relative", 1)

    gamestate_msg = GameState(penalized=False)
    strategy_msg = Strategy(role=Strategy.ROLE_DEFENDER,
                            action=Strategy.ACTION_GOING_TO_BALL,
                            offensive_side=Strategy.SIDE_LEFT)
    position_msg = pose_with_covariance(x=2.0, y=3.0)
    ball_msg = pose_with_covariance(x=8.0, y=9.0)
    obstacle_msg = ObstacleRelativeArray()

    obstacle = ObstacleRelative()
    obstacle.pose.pose.pose.position.x = 4.0
    obstacle.pose.pose.pose.position.y = 5.0
    obstacle.type = ObstacleRelative.ROBOT_CYAN
    obstacle_msg.obstacles.append(obstacle)

    obstacle2 = ObstacleRelative()
    obstacle2.pose.pose.pose.position.x = 1.0
    obstacle2.pose.pose.pose.position.y = 2.0
    obstacle2.type = ObstacleRelative.ROBOT_MAGENTA
    obstacle_msg.obstacles.append(obstacle2)

    while rclpy.ok():
        now = node.get_clock().now().to_msg()
        gamestate_msg.header.stamp = now
        position_msg.header.stamp = now
        obstacle_msg.header.stamp = now
        ball_msg.header.stamp = now

        # @TODO: check if we should use another map frame
        ball_msg.header.frame_id = "map"
        obstacle_msg.header.frame_id = "map"

        gamestate_pub.publish(gamestate_msg)
        strategy_pub.publish(strategy_msg)
        position_pub.publish(position_msg)
        ball_pub.publish(ball_msg)
        obstacle_pub.publish(obstacle_msg)