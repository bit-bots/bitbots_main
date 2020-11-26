#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped


"""
This script simulates precision messages coming from a perfect localization.
"""


if __name__ == "__main__":

    rospy.init_node('sim_localization')

    pose_publisher = rospy.Publisher('pose_with_covariance', PoseWithCovarianceStamped, queue_size=1, latch=True)
    rate = rospy.Rate(20)  # rate of 20 Hz

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    if len(sys.argv) > 1 and sys.argv[1] == '--bad':
        pose.pose.covariance[0] = 100
        pose.pose.covariance[7] = 100
        pose.pose.covariance[35] = 100

    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        pose_publisher.publish(pose)
        rate.sleep()
