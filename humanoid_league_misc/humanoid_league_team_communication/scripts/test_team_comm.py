#!/usr/bin/env python3
"""
This script publishes dummy values for ball, goalpost, position and obstacles for testing the team communication.
"""

import numpy
import rclpy
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion, TransformStamped
from soccer_vision_3d_msgs.msg import Robot, RobotArray
from soccer_vision_attribute_msgs.msg import Robot as RobotAttributes
from tf2_ros import StaticTransformBroadcaster

from bitbots_msgs.msg import GameState, Strategy


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


def base_footprint_transform():
    transform = TransformStamped()
    transform.header.frame_id = "base_footprint"
    transform.child_frame_id = "map"

    return transform


if __name__ == "__main__":
    rclpy.init(args=None)
    node = rclpy.create_node("test_team_comm")
    tf_static_broadcaster = StaticTransformBroadcaster(node)

    gamestate_pub = node.create_publisher(GameState, "gamestate", 1)
    strategy_pub = node.create_publisher(Strategy, "strategy", 1)
    ball_pub = node.create_publisher(PoseWithCovarianceStamped, "ball_position_relative_filtered", 1)
    position_pub = node.create_publisher(PoseWithCovarianceStamped, "pose_with_covariance", 1)
    robots_pub = node.create_publisher(RobotArray, "robots_relative", 1)

    transform = base_footprint_transform()
    gamestate_msg = GameState(penalized=False)
    strategy_msg = Strategy(
        role=Strategy.ROLE_DEFENDER, action=Strategy.ACTION_GOING_TO_BALL, offensive_side=Strategy.SIDE_LEFT
    )
    position_msg = pose_with_covariance(x=2.0, y=3.0)
    ball_msg = pose_with_covariance(x=8.0, y=9.0)
    robots_msg = RobotArray()

    robot = Robot()
    robot.attributes.team = RobotAttributes.TEAM_OWN
    robot.attributes.player_number = 2
    robot.bb.center.position.x = 4.0
    robot.bb.center.position.y = 5.0
    robots_msg.robots.append(robot)

    robot2 = Robot()
    robot2.attributes.team = RobotAttributes.TEAM_OPPONENT
    robot2.attributes.player_number = 3
    robot2.bb.center.position.x = 1.0
    robot2.bb.center.position.y = 2.0
    robots_msg.robots.append(robot2)

    while rclpy.ok():
        now = node.get_clock().now().to_msg()
        transform.header.stamp = now
        gamestate_msg.header.stamp = now
        position_msg.header.stamp = now
        robots_msg.header.stamp = now
        ball_msg.header.stamp = now

        ball_msg.header.frame_id = "base_footprint"
        robots_msg.header.frame_id = "base_footprint"

        tf_static_broadcaster.sendTransform(transform)
        gamestate_pub.publish(gamestate_msg)
        strategy_pub.publish(strategy_msg)
        position_pub.publish(position_msg)
        ball_pub.publish(ball_msg)
        robots_pub.publish(robots_msg)
