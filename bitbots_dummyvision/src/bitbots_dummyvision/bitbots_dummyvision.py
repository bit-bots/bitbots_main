#!/usr/bin/env python2.7
import cv2
import numpy as np
from keras.models import model_from_json
import os
import rospy
from humanoid_league_msgs.msg import BallInImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class DummyVision:
    def __init__(self):
        self.pub_ball = rospy.Publisher("/ball_in_image", BallInImage, queue_size=1)
        self.bridge = CvBridge()

        self.new = False

        with open("src/bitbots_dmummyvision/src/bitbots_dummyvision/model2.json", "r") as j:
            nem = model_from_json(j.read())
        nem.load_weights("src/bitbots_dmummyvision/src/bitbots_dummyvision/model2.ker")
        self.model = nem

        """while not rospy.is_shutdown():
            if self.new:
                #self.work(self.last_img)
                #self.new = False
            else:
                rospy.sleep(0.1)
        """
        rospy.init_node("bitbots_dummyvision")
        rospy.Subscriber("/usb_cam/image_raw", Image, self._image_callback, queue_size=1)

        rospy.spin()

    def work(self, img):
        """
        @param img:
        @type img: Image
        @return:
        """

        print("Recived Image    ")

        ra = self.bridge.imgmsg_to_cv2(img, "bgr8")
        img = cv2.bilateralFilter(ra, 14, 100, 100)
        # mask = cv2.inRange(img, (0, 120, 0), (160, 255, 160))
        # mask = cv2.erode(mask, None, iterations = 2)
        # mask = 255 - mask
        b, g, r = cv2.split(img)
        circles = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, 1, 100,
                                   param1=40, param2=39, minRadius=15, maxRadius=200)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            try:
                for i in circles[0, :]:
                    corp = ra[i[1] - i[2]:i[1] + i[2], i[0] - i[2]:i[0] + i[2]]

                    corp = cv2.resize(corp, (30, 30), interpolation=cv2.INTER_CUBIC)
                    corp.reshape((1,) + corp.shape)
                    #print(corp.shape)

                    p = self.model.predict(np.array([corp]), verbose=0)

                    if p[0][0] < 0.5:
                        c = (0, 255, 0)
                        msg = BallInImage()
                        msg.center.x = i[0]
                        msg.center.y = i[1]
                        msg.diameter = i[2] * 2
                        msg.confidence = p[0]
                        self.pub_ball.publish(msg)
                        #print(i)
                    else:
                        c = (0, 0, 255)
                    #print(p[0][0])
                    # draw the outer circle
                    #cv2.circle(ra, (i[0], i[1]), i[2], c, 2)
                    # draw the center of the circle
                    #cv2.circle(ra, (i[0], i[1]), 2, (0, 0, 255), 3)

            except cv2.SUBDIV2D_PTLOC_ERROR:
                pass

        #cv2.imshow("img", ra)
        #cv2.imshow("mask", g)
        #cv2.waitKey(100)


    def _image_callback(self, img):
        print("test")
        self.last_img = img
        self.new = True
        self.work(img)


if __name__ == "__main__":
    DummyVision()

