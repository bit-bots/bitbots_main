#!/usr/bin/env python3.5
import rospy
import time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("test_pub")
    pub = rospy.Publisher("test", Imu, queue_size=1)

    msg = Imu()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.from_sec(time.time())
        pub.publish(msg)
        rate.sleep()