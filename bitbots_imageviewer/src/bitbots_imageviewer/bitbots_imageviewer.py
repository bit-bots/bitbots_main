#!/usr/bin/env python2.7
import copy
from collections import OrderedDict

import cv2
import os
import rospy
from humanoid_league_msgs.msg import BallInImage, BallsInImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Loadimg:
    def __init__(self):
        rospy.init_node("bitbots_imageviewer")

        rospy.Subscriber("/usb_cam/image_raw", Image, self._image_callback, queue_size=1)
        rospy.Subscriber("/rated_balls_in_image", BallsInImage, self._candidates_callback, queue_size=1)

        self.bridge = CvBridge()
        self.images = OrderedDict()
        self.ball_candidates = OrderedDict()

        while not rospy.is_shutdown():
            images = copy.copy(self.images)
            #print("Waiting for " + str(self.images.keys()))
            for t in images.keys():  # imgages who are wating

                if t in self.ball_candidates:  # Check if all data to draw is there

                    img = images.pop(t)  # get image from queue
                    cans = self.ball_candidates.pop(t)

                    ra = self.bridge.imgmsg_to_cv2(img, "bgr8")
                    if len(cans) > 0:
                        maxcan = max(cans, key=lambda x: x.confidence)
                        for can in cans:
                            i = [0, 0, 0]
                            i[0] = int(can.center.x)
                            i[1] = int(can.center.y)
                            i[2] = int(can.diameter / 2.0)

                            if can == maxcan:
                                c = (255, 0, 0)
                            elif can.confidence >= 0.5:
                                c = (0, 255, 0)
                            else:
                                c = (0, 0, 255)
                                # print(p)
                                # draw the outer circle
                            cv2.circle(ra, (i[0], i[1]), i[2], c, 2)
                            # draw the center of the circle
                            cv2.circle(ra, (i[0], i[1]), 2, (0, 0, 255), 3)
                    cv2.imshow("Image", ra)
                    cv2.waitKey(1)
            rospy.sleep(0.01)

    def _image_callback(self, img):
        self.images[img.header.seq] = img

        if len(self.images) >= 10:
            self.images.popitem(last=False)

    def _candidates_callback(self, balls):
        self.ball_candidates[balls.header.seq] = balls.candidates
        if len(self.ball_candidates) > 5:
            self.ball_candidates.popitem(last=False)


if __name__ == "__main__":
    Loadimg()
