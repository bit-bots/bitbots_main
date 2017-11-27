#!/usr/bin/env python2.7

import rospy
from humanoid_league_msgs.msg import BallInImage, BallsInImage, BallRelative


def run():
    pub_balls = rospy.Publisher("ball_in_image", BallsInImage, queue_size=1)

    counter = -5
    direction = 5
        
    rospy.loginfo("Create Test")
    rospy.init_node("bitbots_testHeadBehaviour")

    rate = rospy.Rate(4)
    rospy.logdebug("L?uft...")
    while not rospy.is_shutdown():
        # Ball in Image
        msg = BallsInImage()
        msg.header.frame_id = "0"
        can = BallInImage()
        can.center.x = counter
        if(counter > 500 or counter < -500):
            direction *= -1
            counter += direction
        else:
            counter += direction
        can.center.y = 100
        can.diameter = 10
        can.confidence = 1
        msg.candidates.append(can)
        
        # Ball relative
        pub_balls.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    run()

