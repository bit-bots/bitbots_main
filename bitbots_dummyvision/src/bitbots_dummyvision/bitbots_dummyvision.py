#!/usr/bin/env python2.7
import cv2
import numpy as np
import os
import rospy
from humanoid_league_msgs.msg import BallInImage, BallsInImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DummyVision:
    def __init__(self):
        self.pub_balls = rospy.Publisher("/ball_candidates", BallsInImage, queue_size=1)
        self.bridge = CvBridge()

        rospy.init_node("bitbots_dummyvision")
        rospy.Subscriber("/usb_cam/image_raw", Image, self._image_callback, queue_size=1)

        rospy.spin()

    def work(self, img):
        """
        @param img:
        @type img: Image
        @return:
        """

        ra = self.bridge.imgmsg_to_cv2(img, "bgr8")
        #bimg = cv2.bilateralFilter(ra, 14, 100, 100)
        bimg = cv2.GaussianBlur(ra, (9, 9), 0)
        # mask = cv2.inRange(img, (0, 120, 0), (160, 255, 160))
        # mask = cv2.erode(mask, None, iterations = 2)
        # mask = 255 - mask
        b, g, r = cv2.split(bimg)
        circles = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, 1, 100,
                                   param1=50, param2=43, minRadius=15, maxRadius=200)

        #build message
        msg = BallsInImage()
        msg.candidates = []
        if circles is not None:
            circles = np.uint16(np.around(circles))

            for i in circles[0, :]:
                #corp = ra[i[1] - i[2]:i[1] + i[2], i[0] - i[2]:i[0] + i[2]]
                can = BallInImage()
                can.center.x = i[0]
                can.center.y = i[1]
                can.diameter = (i[2] * 2) + 3
                msg.candidates.append(can)
                can.header = img.header

        self.pub_balls.publish(msg)

    def _image_callback(self, img):
        self.work(img)


if __name__ == "__main__":
    DummyVision()

