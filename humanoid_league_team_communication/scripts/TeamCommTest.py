#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from humanoid_league_msgs.msg import ObstacleRelativeArray, ObstacleRelative


if __name__ == '__main__':
    rospy.init_node("TeamCommTest")
    ball_pub = rospy.Publisher("ball_relative", PoseWithCovarianceStamped, queue_size=1)
    goal_post_pub = rospy.Publisher("goal_post_relative", PoseWithCovarianceStamped, queue_size=1)
    position_pub = rospy.Publisher("position", PoseWithCovarianceStamped, queue_size=1)
    obstacle_pub = rospy.Publisher("obstacle_relative", ObstacleRelativeArray, queue_size=1)
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x = 2
    obstacle_msg = ObstacleRelativeArray()
    obstacle = ObstacleRelative()
    obstacle.pose.pose.position.x = 2
    obstacle.type = 2
    obstacle_msg.obstacles.append(obstacle)



    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.stamp = rospy.Time.now()
        ball_pub.publish(msg)
        goal_post_pub.publish(msg)
        position_pub.publish(msg)
        obstacle_pub.publish(obstacle_msg)
