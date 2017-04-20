#!/usr/bin/env python2.7
import math
from random import randint

import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from humanoid_league_msgs.msg import BallInImage, BallsInImage, LineSegmentInImage, LineInformationInImage
from sensor_msgs.msg import Image
from cfg import dummyvision_paramsConfig


class DummyVision:
    def __init__(self):
        self.green_min = (0, 110, 37)
        self.green_max = (139, 255, 255)
        self.horizon_x_steps = 30
        self.horizon_y_steps = 30

        self.pub_balls = rospy.Publisher("ball_candidates", BallsInImage, queue_size=1)
        self.pub_lines = rospy.Publisher("line_in_image", LineInformationInImage, queue_size=5)
        self.bridge = CvBridge()

        rospy.Subscriber("image_raw", Image, self._image_callback, queue_size=1)
        rospy.init_node("bitbots_dummyvision")

        self.server = Server(dummyvision_paramsConfig, self.reconfigure)

        rospy.spin()

    def work(self, img):
        """
        @param img:
        @type img: Image
        @return:
        """

        ra = self.bridge.imgmsg_to_cv2(img, "bgr8")
        # bimg = cv2.bilateralFilter(ra, 14, 100, 100)
        bimg = cv2.GaussianBlur(ra, (9, 9), 0)
        mask = cv2.inRange(bimg, self.green_min, self.green_max)
        maskb = cv2.GaussianBlur(mask, (9,9),0)
        # mask = cv2.erode(mask, None, iterations = 2)
        # mask = 255 - mask

        # Horizon
        stepwidth = (ra.shape[1] / (self.horizon_x_steps - 1), ra.shape[0] / (self.horizon_y_steps - 1))  # (x_stepwidth, y_stepwidth)
        horizon = np.empty(self.horizon_x_steps)
        horizon.fill(self.horizon_y_steps)  # worst case as default
        for y in range(self.horizon_y_steps - 1):  # first half step
            if self.check_img(mask[int(y * stepwidth[1]):int((y + 1) * stepwidth[1]), 0:math.ceil(0.5 * stepwidth[0]), ]):
                horizon[0] = y
                break
        for y in range(self.horizon_y_steps - 1):  # last half step
            if self.check_img(mask[int(y * stepwidth[1]):int((y + 1) * stepwidth[1]), int(self.horizon_x_steps - 1.5) * stepwidth[0]:math.ceil((self.horizon_x_steps - 1) * stepwidth[0])]):
                horizon[self.horizon_x_steps - 1] = y
                break
        for x in range(1, self.horizon_x_steps - 1):  # everything in between
            for y in range(self.horizon_y_steps - 1):
                if self.check_img(mask[int(y * stepwidth[1]):int((y + 1) * stepwidth[1]), int((x - 0.5) * stepwidth[0]):math.ceil((x + 0.5) * stepwidth[0])]):
                    horizon[x] = y
                    break

        #b, g, r = cv2.split(bimg)
        circles = cv2.HoughCircles(maskb, cv2.HOUGH_GRADIENT, 1, 100,
                                   param1=20, param2=25, minRadius=7, maxRadius=300)

        # Ball
        msg = BallsInImage()
        msg.header.frame_id = img.header.frame_id
        msg.header.stamp = img.header.stamp
        if circles is not None:
            circles = np.uint16(np.around(circles))

            for i in circles[0, :]:
                if not self.under_horizon(horizon, stepwidth, (i[0], i[1])):
                    cv2.circle(bimg, (i[0], i[1]), i[2], (0, 0, 255))
                    continue
                # corp = ra[i[1] - i[2]:i[1] + i[2], i[0] - i[2]:i[0] + i[2]]
                can = BallInImage()
                cv2.circle(bimg, (i[0], i[1]), i[2], (255, 0, 0))
                can.center.x = i[0]
                can.center.y = i[1]
                can.diameter = (i[2] * 2) + 3
                msg.candidates.append(can)

        # Linepoints
        li = LineInformationInImage()
        li.header.frame_id = img.header.frame_id
        li.header.stamp = img.header.stamp
        for x in range(1000):

            p = randint(0, bimg.shape[0] - 1), randint(0, bimg.shape[1] - 31)

            if self.under_horizon(horizon, stepwidth, p):
                if mask[p[0], p[1]] == 0 and sum(bimg[p[0], p[1]]) > 400:
                    is_ball = False
                    if circles is not None:
                        for b in circles[0, :]:
                            if math.sqrt((b[0] - p[1]) ** 2 + (b[1] - p[0]) ** 2) < b[2] + 15:
                                is_ball = True
                    if is_ball:
                        continue

                    ls = LineSegmentInImage()
                    ls.start.x = p[1]
                    ls.start.y = p[0]
                    ls.end = ls.start
                    li.segments.append(ls)
                    cv2.circle(bimg, (p[1], p[0]), 1, (0, 0, 255))

        # Plotting

        for x in range(len(horizon) - 1):
            cv2.line(bimg, (int(stepwidth[0]) * x, int(stepwidth[1] * horizon[x])), (int(stepwidth[0] * (x + 1)), int(stepwidth[1] * horizon[x + 1])), color=(0, 255, 0))

        cv2.imshow("Image", bimg)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

        self.pub_lines.publish(li)
        self.pub_balls.publish(msg)

    @staticmethod
    def check_img(subim):
        m = np.average(np.average(subim, axis=0), axis=0)
        return m > 100

    @staticmethod
    def under_horizon(horizon, stepwidth, i):
        """returns true if point i is under the horizon"""
        try:
            return horizon[(i[0] // stepwidth[0])] < i[1] / stepwidth[1]
        except:
            return False

    def _image_callback(self, img):
        print("in:", img.header.stamp.to_time())
        self.work(img)

    def reconfigure(self, config, level):
        # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
        self.green_min = (config["b_min"], config["g_min"], config["r_min"])
        self.green_max = (config["b_max"], config["g_max"], config["r_max"])
        self.horizon_x_steps = config["x_stepcount"]
        self.horizon_y_steps = config["y_stepcount"]

        # Return the new variables.
        return config


if __name__ == "__main__":
    DummyVision()
