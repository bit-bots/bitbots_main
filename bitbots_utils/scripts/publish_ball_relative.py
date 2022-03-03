#!/usr/bin/env python3
# -*- coding: utf8 -*-

import rclpy
from rclpy.node import Node
import sys
import random
from humanoid_league_msgs.msg import PoseWithCertainty, PoseWithCertaintyArray

delta = 1
max_x = 10
min_x = 0.3
max_y = 10
min_y = -10
max_z = 0
min_z = 0

if __name__ == "__main__":
    rclpy.init(args=None)
    balls_relative_publisher = self.create_publisher(PoseWithCertaintyArray, "balls_relative", 10, tcp_nodelay=True)

    x = random.uniform(min_x, max_x) / 4
    y = random.uniform(min_y, max_y) / 4
    z = random.uniform(min_z, max_z) / 4

    while rclpy.ok():
        ball_msg = PoseWithCertainty()
        if len(sys.argv) is in [3, 4]:
            ball_msg.pose.pose.position.x = float(sys.argv[1])
            ball_msg.pose.pose.position.y = float(sys.argv[2])
            if len(sys.argv) == 4:
                ball_msg.pose.pose.position.z= float(sys.argv[3])
        else:
            ball_msg.ball_relative.x = x
            ball_msg.ball_relative.y = y
            ball_msg.ball_relative.z = z

            # New position for next step
            x = max(min_x, min(x + delta * random.uniform(-1, 1), max_x))
            y = max(min_y, min(y + delta * random.uniform(-1, 1), max_y))
            z = max(min_z, min(z + delta * random.uniform(-1, 1), max_z))

        ball_msg.confidence = 1.0

        balls_msg = PoseWithCertaintyArray()
        balls_msg.header.stamp = self.get_clock().now()
        balls_msg.poses = [ball_msg]
        balls_relative_publisher.publish(balls_msg)

        rospy.sleep(0.5)
0.5)
