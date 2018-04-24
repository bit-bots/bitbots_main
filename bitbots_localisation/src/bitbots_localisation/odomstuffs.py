#!/usr/bin/env python2.7
import rospy


from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
import tf


class TFWorld(object):
    def __init__(self):
        rospy.init_node("odommap")

        br = tf.TransformBroadcaster()

        transform = TransformStamped()
        transform.header.frame_id = "odom"
        transform.header.stamp = rospy.Time.now()
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = 0
        transform.transform.translation.y = 0
        transform.transform.translation.z = 0
        transform.transform.rotation.x = 0
        transform.transform.rotation.y = 0
        transform.transform.rotation.z = 0
        transform.transform.rotation.w = 1
        while(True):
            br.sendTransformMessage(transform)
            rospy.sleep(rospy.Duration(0.1))


if __name__ == "__main__":
    TFWorld()
