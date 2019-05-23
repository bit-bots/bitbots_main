#!/usr/bin/env python2.7
import cv2
import os
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Loadimg:
    def __init__(self):
        rospy.init_node("bitbots_imageloader")
        rospy.loginfo("started")
        self.pub_im = rospy.Publisher(rospy.get_param("/imageloader/topic", "image_raw"), Image, queue_size=1)
        self.bridge = CvBridge()

        path = rospy.get_param("/imageloader/load_from")

        nr = 1000
        listdir = list(os.listdir(path))

        rate = rospy.Rate(rospy.get_param("/imageloader/framerate", 25))

        img_id = 3
        while not rospy.is_shutdown():
            for im in sorted(listdir)[:nr]:
                rospy.loginfo(im)
                ra = cv2.imread(os.path.join(path, im))

                msg = self.bridge.cv2_to_imgmsg(ra, "bgr8")
                msg.header.frame_id = "/camera_link"
                img_id += 1
                msg.header.stamp = rospy.get_rostime()
                self.pub_im.publish(msg)
                if rospy.is_shutdown():
                    break;
                rate.sleep()


if __name__ == "__main__":
    Loadimg()
