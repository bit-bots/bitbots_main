#!/usr/bin/env python3

import rospy
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

    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        pose_publisher.publish(pose)
        rate.sleep()
