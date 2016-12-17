#!/usr/bin/env python2.7
import cv2
import numpy as np
from keras.models import model_from_json
import os
import rospy
from humanoid_league_msgs.msg import BallInImage, BallsInImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Classifier:
    def __init__(self):
        self.pub_ball = rospy.Publisher("/ball_in_image", BallInImage, queue_size=1)
        self.bridge = CvBridge()
        self.last_img = None
        self.new_can = False

        with open("src/bitbots_ballclassifier/src/bitbots_ballclassifier/model2.json", "r") as j:
            nem = model_from_json(j.read())
        nem.load_weights("src/bitbots_ballclassifier/src/bitbots_ballclassifier/model2.ker")
        self.model = nem

        rospy.init_node("bitbots_ball_classifier")
        rospy.Subscriber("/usb_cam/image_raw", Image, self._image_callback, queue_size=1)
        rospy.Subscriber("/ball_candidates", BallsInImage, self._candidates_callback, queue_size=1)
        # TODO Lineage problem

        while not rospy.is_shutdown():
            if self.new_can:
                self.work()
                self.new_can = False
            else:
                rospy.sleep(0.01)

    def work(self):
        """
        @param img:
        @type img: Image
        @return:
        """

        print("Recived Image   classifiying ")
        if self.last_img is None:
            return

        ra = self.bridge.imgmsg_to_cv2(self.last_img, "bgr8")

        if self.candidates is not None:
            try:
                for i in self.candidates:
                    corp = ra[i[1] - i[2]:i[1] + i[2], i[0] - i[2]:i[0] + i[2]]

                    corp = cv2.resize(corp, (30, 30), interpolation=cv2.INTER_CUBIC)
                    corp.reshape((1,) + corp.shape)

                    p = self.model.predict(np.array([corp]), verbose=0)

                    if p[0][0] < 0.5:
                        c = (0, 255, 0)
                        msg = BallInImage()
                        msg.center.x = i[0]
                        msg.center.y = i[1]
                        msg.diameter = i[2] * 2
                        msg.confidence = p[0]
                        self.pub_ball.publish(msg)

                    else:
                        c = (0, 0, 255)
                    # print(p[0][0])
                    # draw the outer circle
                    cv2.circle(ra, (i[0], i[1]), i[2], c, 2)
                    # draw the center of the circle
                    cv2.circle(ra, (i[0], i[1]), 2, (0, 0, 255), 3)

            except cv2.error:
                pass

        cv2.imshow("img", ra)
        cv2.waitKey(1)

    def _image_callback(self, img):
        self.last_img = img

    def _candidates_callback(self, candidates):
        self.candidates = []
        for can in candidates.candidates:
            self.candidates.append((int(can.center.x),
                                    int(can.center.y),
                                    int(can.diameter / 2.0)))
        self.new_can = True
        print("callback")



if __name__ == "__main__":
    Classifier()

