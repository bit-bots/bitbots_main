#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

if __name__ == '__main__':
    rospy.init_node('dummy_imu')

    pub = rospy.Publisher('/imu/data', Imu, queue_size=1)

    msg = Imu()
    msg.header.frame_id = 'imu'
    msg.orientation.w = 1

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        r.sleep()
