#!/usr/bin/env python2.7
"""
Command line tool to publish balls on the /goal_in_image topic
"""
import rospy
from humanoid_league_msgs.msg import GoalInImage, GoalPartsInImage, GoalPostInImage
import sys
import signal
from geometry_msgs.msg import Point, PointStamped


pub = None
def _signal_term_handler(signal, frame):
    rospy.logerr('User Keyboard interrupt')
    sys.exit(0)

def point_cb(msg):
    global pub
    gi = GoalInImage()
    gi.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
    post = GoalPostInImage()
    post.foot_point = Point(msg.point.x, msg.point.y, 0)
    post.confidence = 1
    gi.left_post = post
    gi.confidence = 1
    pub.publish(gi)


if __name__ == "__main__":
    # handle keyboard interrupts
    signal.signal(signal.SIGINT, _signal_term_handler)

    rospy.init_node("goal_tester")
    pub = rospy.Publisher("goal_in_image", GoalInImage, queue_size=10)
    rospy.Subscriber("/image_raw/screenpoint", PointStamped, point_cb)

    while True:
        x_str = raw_input("foot_point left x:")
        try:
            x = int(x_str)
        except ValueError:
            print("try again")
            continue
        y_str = raw_input("foot_point left y:")
        try:
            y = int(y_str)
        except ValueError:
            print("try again")
            continue

        gi = GoalInImage()
        gi.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
        post = GoalPostInImage()
        post.foot_point = Point(x, y, 0)
        post.confidence = 1
        gi.left_post = post
        gi.confidence = 1

        x_str = raw_input("foot_point right x:")
        try:
            x = int(x_str)
        except ValueError:
            print("try again")
            continue
        y_str = raw_input("foot_point right y:")
        try:
            y = int(y_str)
        except ValueError:
            print("try again")
            continue

        post2 = GoalPostInImage()
        post2.foot_point = Point(x, y, 0)
        gi.right_post = post2


        pub.publish(gi)

