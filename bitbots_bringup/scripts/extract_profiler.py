#!/usr/bin/env python3

import rospy
from swri_profiler_msgs.msg import ProfileDataArray
from bitbots_msgs.msg import ProfileDataStamped

rospy.init_node("extract_profiler")
pub = rospy.Publisher("/profiler/single_data", ProfileDataStamped, queue_size=1)
def cb(msg):
    pub.publish(ProfileDataStamped(msg.header, msg.rostime_stamp, msg.data[0]))

rospy.Subscriber("/profiler/data", ProfileDataArray, cb)
rospy.spin()