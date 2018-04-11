#!/usr/bin/env python2.7
import rospy
from humanoid_league_msgs.msg import BallsInImage, BallInImage



rospy.init_node("transform_tester")
pub = rospy.Publisher("ball_in_image", BallsInImage,queue_size=10)

while True:
    x = int(raw_input("x:"))
    y = int(raw_input("y:"))


    bi = BallsInImage()
    bi.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
    ball = BallInImage()
    ball.confidence = 1
    ball.center.x = x
    ball.center.y = y
    ball.diameter = 0.13

    bi.candidates.append(ball)

    pub.publish(bi)