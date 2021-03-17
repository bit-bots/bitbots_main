#!/usr/bin/env python3

"""
This script publishes dummy values for ball, goalpost, position and obstacles for testing the team communication.
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from humanoid_league_msgs.msg import ObstacleRelativeArray, ObstacleRelative, PoseWithCertaintyArray, PoseWithCertainty


if __name__ == '__main__':
    rospy.init_node("TeamCommTest")
    ball_pub = rospy.Publisher("balls_relative", PoseWithCertaintyArray, queue_size=1)
    goal_post_pub = rospy.Publisher("goal_posts_relative", PoseWithCertaintyArray, queue_size=1)
    position_pub = rospy.Publisher("pose_with_certainty", PoseWithCertainty, queue_size=1)
    obstacle_pub = rospy.Publisher("obstacles_relative", ObstacleRelativeArray, queue_size=1)
    position_msg = PoseWithCertainty()
    position_msg.pose.pose.position.x = 2
    position_msg.confidence = 0.7
    obstacle_msg = ObstacleRelativeArray()
    obstacle = ObstacleRelative()
    obstacle.pose.pose.pose.position.x = 4
    obstacle.type = 2
    obstacle_msg.obstacles.append(obstacle)
    ball_msg = PoseWithCertaintyArray()
    ball = PoseWithCertainty()
    ball.confidence = 1.0
    ball_position = PoseWithCovariance()
    ball_position.pose.position.x = 3
    ball.pose = ball_position
    ball_msg.poses.append(ball)
    goal_msg = PoseWithCertaintyArray()
    goalpost = PoseWithCertainty()
    goalpost.confidence = 1.0
    goalpost_position = PoseWithCovariance()
    goalpost_position.pose.position.x = 10
    goalpost.pose = goalpost_position
    goal_msg.poses.append(goalpost)

    while not rospy.is_shutdown():
        obstacle_msg.header.stamp = rospy.Time.now()
        ball_msg.header.stamp = rospy.Time.now()
        goal_msg.header.stamp = rospy.Time.now()
        ball_pub.publish(ball_msg)
        goal_post_pub.publish(goal_msg)
        position_pub.publish(position_msg)
        obstacle_pub.publish(obstacle_msg)
