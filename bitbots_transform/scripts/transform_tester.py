#!/usr/bin/env python2.7
import rospy
from humanoid_league_msgs.msg import BallsInImage, BallInImage
import sys
import signal



def signal_term_handler(signal, frame):
  rospy.logerr('User Keyboard interrupt')
  sys.exit(0)

if __name__ == "__main__":
    # handle keyboard interrupts
    signal.signal(signal.SIGINT, signal_term_handler)
    
    rospy.init_node("transform_tester")
    pub = rospy.Publisher("ball_in_image", BallsInImage,queue_size=10)

    while True:
        x_str = raw_input("x:")
        y_str = raw_input("y:")


        try:
            x = int(x_str)
            y = int(y_str)
        except ValueError:
            print("try again, without fucking this time up please")
            continue

        bi = BallsInImage()
        bi.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
        ball = BallInImage()
        ball.confidence = 1
        ball.center.x = x
        ball.center.y = y
        ball.diameter = 0.13

        bi.candidates.append(ball)

        pub.publish(bi)

