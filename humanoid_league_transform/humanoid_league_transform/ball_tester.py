#!/usr/bin/env python3
"""
Command line tool to publish balls on the /ball_in_image topic
"""
import rclpy
from rclpy.node import Node
from humanoid_league_msgs.msg import BallInImage, BallInImageArray
import sys
import signal



def _signal_term_handler(signal, frame):
    self.get_logger().error('User Keyboard interrupt')
    sys.exit(0)


if __name__ == "__main__":
    # handle keyboard interrupts
    signal.signal(signal.SIGINT, _signal_term_handler)

    rclpy.init(args=None)
    pub = self.create_publisher(BallInImageArray, "balls_in_image", 10)

    while True:
        x_str = input("x:")
        try:
            x = int(x_str)
        except ValueError:
            print("try again")
            continue
        y_str = input("y:")
        try:
            y = int(y_str)
        except ValueError:
            print("try again")
            continue

        ba = BallInImageArray()
        ba.header.stamp = rospy.get_rostime() - Duration(seconds=0.2)
        ball = BallInImage()
        ball.confidence = 1
        ball.center.x = x
        ball.center.y = y
        ball.diameter = 0.13

        ba.candidates.append(ball)

        pub.publish(ba)

