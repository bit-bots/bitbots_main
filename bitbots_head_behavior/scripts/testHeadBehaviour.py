#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
from humanoid_league_msgs.msg import BallInImage, BallInImageArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def run():
    node = Node("testHeadBehaviour")
    pub_ball = node.create_publisher(BallInImage, "ball_in_image", 1)
    pub_hmg = node.create_publisher(JointTrajectory, "head_motor_goals", 1)

    hmg = JointTrajectory()
    goal = JointTrajectoryPoint()
    goal.positions = [0, 0]
    goal.velocities = [0, 0]
    hmg.points = [goal]


    counter = 320
    direction = 1

    node.get_logger().info("Create Test")
    rclpy.init(args=None)
    pub_hmg.publish(hmg)

    rate = node.create_rate(4)
    node.get_logger().debug("Laeuft...")
    while rclpy.ok():
        # Ball in Image
        ball = BallInImage()
        ball.center.x = counter
        if(counter > 340 or counter < 300):
            direction *= -1
            counter += direction
        else:
            counter += direction
        ball.center.y = 200
        ball.diameter = 10
        ball.confidence = 1
        balls = BallInImageArray()
        balls.candidates.append(ball)

        pub_ball.publish(balls)
        node.get_logger().info("Published ball: %s" % counter)
        rate.sleep()

if __name__ == "__main__":
    run()

