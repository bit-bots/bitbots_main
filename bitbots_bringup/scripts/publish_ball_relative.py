#!/usr/bin/env python3
# -*- coding: utf8 -*-

import rospy
import sys
import random
from humanoid_league_msgs.msg import BallRelative

delta = 1
max_x = 10
min_x = 0.3
max_y = 10
min_y = -10

if __name__ == "__main__":
    rospy.init_node("ball_relative_publisher")
    ball_relative_publisher = rospy.Publisher("ball_relative", BallRelative, queue_size=10, tcp_nodelay=True)
    ball_msg = BallRelative()
    x = random.uniform(min_x, max_x) / 4
    y = random.uniform(min_y, max_y) / 4

    while not rospy.is_shutdown():
        if len(sys.argv) == 3:
            ball_msg.ball_relative.x = float(sys.argv[1])
            ball_msg.ball_relative.y = float(sys.argv[2])
        else:
            ball_msg.ball_relative.x = x
            ball_msg.ball_relative.y = y
            x = max(min_x, min(x + delta * random.uniform(-1, 1), max_x))
            y = max(min_y, min(y + delta * random.uniform(-1, 1), max_y))
        ball_msg.ball_relative.z = 0
        ball_msg.confidence = 1
        ball_msg.header.stamp = rospy.Time.now()
        rospy.loginfo(ball_msg)
        ball_relative_publisher.publish(ball_msg)
        rospy.sleep(0.5)
