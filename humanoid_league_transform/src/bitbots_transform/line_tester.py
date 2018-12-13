#!/usr/bin/env python2.7
"""
Command line tool to publish lines on the /line_in_image topic
"""
import rospy
from humanoid_league_msgs.msg import LineInformationInImage, LineSegmentInImage
import sys
import signal


def _signal_term_handler(signal, frame):
    rospy.logerr('User Keyboard interrupt')
    sys.exit(0)


if __name__ == "__main__":
    # handle keyboard interrupts
    signal.signal(signal.SIGINT, _signal_term_handler)

    rospy.init_node("line_tester")
    pub = rospy.Publisher("line_in_image", LineInformationInImage, queue_size=10)

    x_start_str = raw_input("x_start:")
    try:
        x_start = int(x_start_str)
    except ValueError:
        print("try again, without fucking this time up please")
        #continue

    y_start_str = raw_input("y_start:")
    try:
        y_start = int(y_start_str)
    except ValueError:
        print("try again, without fucking this time up please")
       # continue

    x_end_str = raw_input("x_end:")
    try:
        x_end = int(x_end_str)
    except ValueError:
        print("try again, without fucking this time up please")
       # continue

    y_end_str = raw_input("y_end:")
    try:
        y_end = int(y_end_str)
    except ValueError:
        print("try again, without fucking this time up please")
       # continue

    while True:
        li = LineInformationInImage()
        # press play in simulation first!
        li.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
        seg = LineSegmentInImage()
        seg.start.x = x_start
        seg.start.y = y_start
        seg.end.x = x_end
        seg.end.y = y_end
        seg.confidence = 1.0
        seg.start_width = 5
        seg.end_width = 5
        li.segments.append(seg)

        pub.publish(li)

