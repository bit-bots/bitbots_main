#!/usr/bin/env python2.7

import rospy
from humanoid_league_msgs.msg import BallInImage, BallsInImage, BallRelative
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def run():
    pub_balls = rospy.Publisher("ball_in_image", BallsInImage, queue_size=1)
    pub_hmg = rospy.Publisher("head_motor_goals", JointTrajectory, queue_size=1)

    hmg = JointTrajectory()
    goal = JointTrajectoryPoint()
    goal.positions = [0, 0]
    goal.velocities = [0, 0]
    hmg.points = [goal]


    counter = 320
    direction = 1
        
    rospy.loginfo("Create Test")
    rospy.init_node("bitbots_testHeadBehaviour")
    pub_hmg.publish(hmg)

    rate = rospy.Rate(4)
    rospy.logdebug("Laeuft...")
    while not rospy.is_shutdown():
        # Ball in Image
        ball_in_image_msg = BallsInImage()
        can = BallInImage()
        can.center.x = counter
        if(counter > 340 or counter < 300):
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

