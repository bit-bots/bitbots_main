#!/usr/bin/env python2.7
import cv2
import os
import rospy
from humanoid_league_msgs.msg import BallInImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Loadimg:
    def __init__(self):
        rospy.init_node("bitbots_imageloader")
        print("started")
        self.pub_im = rospy.Publisher("/raw_image", Image, queue_size=1)
        self.bridge = CvBridge()
        path = rospy.get_param("/imageloader/load_from", "/home/martin/Schreibtisch/ds_x/ds2")

        listdir = list(os.listdir(path))

        for im in sorted(listdir)[:100]:
            print(im)
            ra = cv2.imread(os.path.join(path, im))

            msg = self.bridge.cv2_to_imgmsg(ra, "bgr8")

            self.pub_im.publish(msg)

            rospy.sleep(0.2)


if __name__ == "__main__":
    Loadimg()
