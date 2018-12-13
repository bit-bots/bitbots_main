#!/usr/bin/env python2.7
import rospy


from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
import tf


class TFWorld(object):
    def __init__(self):
        rospy.init_node("map")
        #rospy.Subscriber("/gazebo/model_states", ModelStates, self._callback, queue_size=1)
        br = tf.TransformBroadcaster()
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            transform = TransformStamped()
            transform.header.frame_id = "map"
            transform.header.stamp = rospy.Time.now()
            transform.child_frame_id = "odom"

            transform.transform.translation.x = 0
            transform.transform.translation.y = 0
            transform.transform.translation.z = 0
            transform.transform.rotation.x = 0
            transform.transform.rotation.y = 0
            transform.transform.rotation.z = 0
            transform.transform.rotation.w = 1
            br.sendTransformMessage(transform)
            r.sleep()

        rospy.spin()


if __name__ == "__main__":
    TFWorld()
