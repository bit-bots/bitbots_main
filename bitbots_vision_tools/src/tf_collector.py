#! /usr/bin/env python2


from __future__ import print_function
from std_msgs.msg import Header
import tf2_ros
import rospy
import yaml
import os

class TFCollector(object):
    def __init__(self):

        rospy.init_node("tf_collector")
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/extract_time", Header, self.time_cb, queue_size=100)

        self.n = 0
        directory = "/media/usbstick/" + rospy.get_param("img_folder")
        if not os.path.exists(directory):
            os.makedirs(directory)

        self.f = open(directory + "/" + str(rospy.get_rostime().to_sec()) + ".yaml", "w")

        while not rospy.is_shutdown():
            rospy.Rate(0.1).sleep()

        self.f.close()

    def time_cb(self, msg):
        tf = self.tf_buffer.lookup_transform("base_link", "camera", msg.stamp, timeout=rospy.Duration(0.2))
        rospy.logwarn("yaaay")
        rospy.loginfo(str(msg.seq) + "\n" + yaml.dump(tf), file=self.f)
        self.n += 1

if __name__ == "__main__":
    TFCollector()
