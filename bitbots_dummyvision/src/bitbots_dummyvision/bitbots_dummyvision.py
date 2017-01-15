#!/usr/bin/env python2.7
from random import randint
import math

import cv2
import numpy as np
import rospy
from humanoid_league_msgs.msg import BallInImage, BallsInImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DummyVision:
    def __init__(self):
        self.pub_balls = rospy.Publisher("/ball_candidates", BallsInImage, queue_size=1)
        self.bridge = CvBridge()

        rospy.Subscriber("/usb_cam/image_raw", Image, self._image_callback, queue_size=1)
        rospy.init_node("bitbots_dummyvision")

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
        mask = cv2.inRange(ra, (0, 120, 0), (160, 255, 160))
        # mask = cv2.erode(mask, None, iterations = 2)
        # mask = 255 - mask

        # Horizion
        horizonbase = np.zeros((ra.shape[0] / 30, ra.shape[1] / 30))
        for x in range(ra.shape[0] / 30):
            for y in range(ra.shape[1] / 30):
                horizonbase[x, y] = self.check_img(mask[x * 30:(x + 1) * 30, y * 30:(y + 1) * 30])

        horizon = []
        for col in range(horizonbase.shape[1]):
            horizon.append(list(horizonbase[:, col]).index(1) * 30)


        b, g, r = cv2.split(bimg)
        circles = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, 1, 100,
                                   param1=50, param2=43, minRadius=15, maxRadius=200)

        #Ball
        msg = BallsInImage()
        msg.header.frame_id = img.header.frame_id
        msg.header.stamp = img.header.stamp
        if circles is not None:
            circles = np.uint16(np.around(circles))

            for i in circles[0, :]:
                if not self.under_horizon(horizon, (i[1], i[0])):
                    #cv2.circle(bimg, (i[0], i[1]), i[2], (0, 0, 255))
                    continue
                #corp = ra[i[1] - i[2]:i[1] + i[2], i[0] - i[2]:i[0] + i[2]]
                can = BallInImage()
                #cv2.circle(bimg, (i[0], i[1]), i[2], (255,0,0))
                can.center.x = i[0]
                can.center.y = i[1]
                can.diameter = (i[2] * 2) + 3
                can.header.frame_id = img.header.frame_id
                can.header.stamp = img.header.stamp
                msg.candidates.append(can)

        # Linepoints
        imagepoints= []
        for x in range(1000):

            p = randint(0, bimg.shape[0]-1), randint(0, bimg.shape[1]-31)

            if self.under_horizon(horizon, p):
                if mask[p[0], p[1]] == 0 and sum(bimg[p[0], p[1]]) > 400:
                    is_ball = False
                    if circles is not None:
                        for b in circles[0, :]:
                            if math.sqrt((b[0]-p[1])**2 + (b[1]-p[0])**2) < b[2] + 15:
                                is_ball = True
                    if is_ball:
                        continue

                    imagepoints.append(p)
                    #cv2.circle(bimg, (p[1], p[0]), 1, (0, 0, 255))




        # Plotting
        """
        for x in range(len(horizon)-1):
            cv2.line(bimg, (30*x + 15, horizon[x]),(30*(x+1) +15, horizon[x+1]), color=(0,255,0))

        cv2.imshow("Image", bimg)
        #cv2.imshow("Mask", mask)
        cv2.waitKey(1)
        """
        self.pub_balls.publish(msg)

    @staticmethod
    def check_img(subim):
        m = np.average(np.average(subim, axis=0), axis=0)
        return m > 100

    @staticmethod
    def under_horizon(horizon, i):
        return horizon[(i[1] // 30)] < i[0]

    def _image_callback(self, img):
        self.work(img)


if __name__ == "__main__":
    DummyVision()

