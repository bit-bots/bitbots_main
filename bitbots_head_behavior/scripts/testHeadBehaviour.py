#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from humanoid_league_msgs.msg import BallInImage, BallRelative, BallsInImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def run():
    pub_ball = rospy.Publisher("ball_in_image", BallsInImage, queue_size=1)
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
        ball = BallInImage()
        ball.center.x = counter
        if(counter > 340 or counter < 300):
            direction *= -1
            counter += direction
        else:
            counter += direction
        ball.center.y = 200
        ball.diameter = 10
        ball.confidence = 1
        balls = BallsInImage()
        balls.candidates.append(ball)

        pub_ball.publish(balls)
        rospy.loginfo("Published ball: %s" % counter)
        rate.sleep()

if __name__ == "__main__":
    run()

