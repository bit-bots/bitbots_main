#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from swri_profiler_msgs.msg import ProfileDataArray
from bitbots_msgs.msg import ProfileDataStamped

"""
This script extracts data from the SWRI profiler to later use it in combination with the rosbag_to_pandas script.
"""
rclpy.init(args=None)
pub = self.create_publisher(ProfileDataStamped, "/profiler/single_data", 1)


def cb(msg):
    pub.publish(ProfileDataStamped(msg.header, msg.rostime_stamp, msg.data[0]))


rospy.Subscriber("/profiler/data", ProfileDataArray, cb)
rclpy.spin(self)
