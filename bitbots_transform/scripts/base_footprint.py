#!/usr/bin/env python2.7
import tf2_ros
from tf import transformations
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped


class BaseFootprintBroadcaster:
    def __init__(self):
        rospy.init_node("base_footprint_broadcaster")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("imu/data", Imu, self.callback_imu)
        self.tf = TransformStamped()
        self.tf.header.frame_id = "base_link"
        self.tf.child_frame_id = "base_footprint"
        self.tf.transform.translation.x = 0.0
        self.tf.transform.translation.y = 0.0

        rospy.spin()

    def callback_imu(self, msg):
        self.tf.header.stamp = msg.header.stamp
        try:
            tf_right = self.tf_buffer.lookup_transform("base_link", "r_sole", msg.header.stamp, timeout=rospy.Duration(0.1))
            tf_left = self.tf_buffer.lookup_transform("base_link", "l_sole", msg.header.stamp, timeout=rospy.Duration(0.1))
        except tf2_ros.LookupException:
            rospy.logwarn("still waiting for transforms")
            return

        if tf_right.transform.translation.z < tf_left.transform.translation.z:
            self.tf.transform.translation.z = tf_right.transform.translation.z

        else:
            self.tf.transform.translation.z = tf_right.transform.translation.z

        self.tf.transform.rotation = msg.orientation

        """
        self.tf.transform.rotation.x = msg.orientation.x
        self.tf.transform.rotation.y = msg.orientation.y
        self.tf.transform.rotation.z = msg.orientation.z
        self.tf.transform.rotation.w = msg.orientation.w"""


        self.tf_broadcaster.sendTransform(self.tf)


if __name__ == "__main__":
    BaseFootprintBroadcaster()
