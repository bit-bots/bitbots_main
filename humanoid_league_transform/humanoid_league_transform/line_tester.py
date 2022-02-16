#!/usr/bin/env python3
"""
Command line tool to publish lines on the /line_in_image topic
"""
import rclpy
from rclpy.node import Node
from humanoid_league_msgs.msg import LineInformationInImage, LineSegmentInImage
import sys
import signal


def _signal_term_handler(signal, frame):
    self.get_logger().error('User Keyboard interrupt')
    sys.exit(0)


if __name__ == "__main__":
    # handle keyboard interrupts
    signal.signal(signal.SIGINT, _signal_term_handler)

    rclpy.init(args=None)
    pub = self.create_publisher(LineInformationInImage, "line_in_image", 10)

    x_start_str = input("x_start:")
    try:
        x_start = int(x_start_str)
    except ValueError:
        print("try again")
        #continue

    y_start_str = input("y_start:")
    try:
        y_start = int(y_start_str)
    except ValueError:
        print("try again")
       # continue

    x_end_str = input("x_end:")
    try:
        x_end = int(x_end_str)
    except ValueError:
        print("try again")
       # continue

    y_end_str = input("y_end:")
    try:
        y_end = int(y_end_str)
    except ValueError:
        print("try again")
       # continue

    while True:
        li = LineInformationInImage()
        # press play in simulation first!
        li.header.stamp = rospy.get_rostime() - Duration(seconds=0.2)
        seg = LineSegmentInImage()
        seg.start.x = x_start
        seg.start.y = y_start
        seg.end.x = x_end
        seg.end.y = y_end
        seg.confidence = 1.0
        li.segments.append(seg)

        pub.publish(li)

