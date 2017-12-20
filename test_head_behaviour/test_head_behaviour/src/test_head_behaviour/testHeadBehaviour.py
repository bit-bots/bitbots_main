#!/usr/bin/env python2.7

import rospy
from humanoid_league_msgs.msg import BallInImage, BallsInImage, BallRelative, BallRelative


def run():
    pub_balls = rospy.Publisher("ball_in_image", BallsInImage, queue_size=1)

    counter = 320
    direction = 5
        
    rospy.loginfo("Create Test")
    rospy.init_node("bitbots_testHeadBehaviour")

    rate = rospy.Rate(4)
    rospy.logdebug("Laeuft...")
    while not rospy.is_shutdown():
        # Ball in Image
        ball_in_image_msg = BallsInImage()
        can = BallInImage()
        can.center.x = counter
        if(counter > 600 or counter < 40):
            direction *= -1
            counter += direction
        else:
            counter += direction
        can.center.y = 200
        can.diameter = 10
        can.confidence = 1
        ball_in_image_msg.candidates.append(can)

        pub_balls.publish(ball_in_image_msg)
        rate.sleep()

if __name__ == "__main__":
    run()

