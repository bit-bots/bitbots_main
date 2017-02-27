#!/usr/bin/env python2.7
import os
from collections import OrderedDict

import cv2
import numpy as np
from cv_bridge import CvBridge
from keras.models import model_from_json
from sensor_msgs.msg import Image, genpy

import rospy
from humanoid_league_msgs.msg import BallInImage, BallsInImage


class Classifier:
    def __init__(self):
        self.pub_ball = rospy.Publisher("ball_in_image", BallInImage, queue_size=1)
        self.pub_rated_ball = rospy.Publisher("rated_balls_in_image", BallsInImage, queue_size=1)

        path = rospy.get_param("/bitbots_ballclassifier/runpath", "")

        self.bridge = CvBridge()
        self.last_imgs = OrderedDict()
        self.new_can = False
        self.latest_image_id = None

        with open(os.path.join(path, "models/model4.json"), "r") as j:
            nem = model_from_json(j.read())
        nem.load_weights(os.path.join(path, "models/model4.ker"))
        self.model = nem

        rospy.init_node("bitbots_ball_classifier")
        rospy.Subscriber("image_raw", Image, self._image_callback, queue_size=1)
        rospy.Subscriber("ball_candidates", BallsInImage, self._candidates_callback, queue_size=1)

        while not rospy.is_shutdown():
            if self.new_can and self.latest_image_id is not None:
                self.work()
                self.new_can = False
            else:
                rospy.sleep(0.01)

    def work(self):
        #print(self.last_imgs.keys())
        #print(self.latest_image_id)
        img = self.last_imgs.pop(self.latest_image_id, None)
        if img is None:
            rospy.logerr("Candidate " + str(self.latest_image_id) + " was too late")
            return
        ra = self.bridge.imgmsg_to_cv2(img, "bgr8")

        quality = []
        for i in self.candidates:
            x_b = max(i[1] - i[2] - 3, 0)
            x_e = min(i[1] + i[2] + 3, ra.shape[0])
            y_b = max(i[0] - i[2] - 3, 0)
            y_e = min(i[0] + i[2] + 3, ra.shape[1])
            corp = ra[x_b:x_e, y_b:y_e]

            corp = cv2.resize(corp, (30, 30), interpolation=cv2.INTER_CUBIC)
            corp.reshape((1,) + corp.shape)

            p = self.model.predict(np.array([corp]), verbose=0)
            print(p[0][0])
            if p[0][0] < 0.5:
                print("ignored ball candidate (bad rating)")
                continue

            msg = BallInImage()
            msg.center.x = i[0]
            msg.center.y = i[1]
            msg.diameter = i[2] * 2
            msg.confidence = p[0][0]
            msg.header.frame_id = img.header.frame_id
            msg.header.stamp = img.header.stamp
            quality.append(msg)

        if len(quality) > 0:
            self.pub_ball.publish(max(quality, key=lambda x: x.confidence))
            rb = BallsInImage()
            rb.candidates = quality
            rb.header.frame_id = img.header.frame_id
            rb.header.stamp = img.header.stamp
            self.pub_rated_ball.publish(rb)
        else:
            b = BallInImage()
            b.header.frame_id = img.header.frame_id
            b.header.stamp = img.header.stamp
            self.pub_ball.publish(b)
            bs = BallsInImage()
            bs.header.frame_id = img.header.frame_id
            bs.header.stamp = img.header.stamp
            self.pub_rated_ball.publish(bs)

    def _image_callback(self, img):
        self.last_imgs[img.header.stamp] = img
        print("img in : " , img.header.stamp.to_time())

        if len(self.last_imgs) > 15:
            self.last_imgs.popitem(last=False)

    def _candidates_callback(self, candidates):
        self.candidates = []
        #print("can: ",  candidates.header.stamp)
        for can in candidates.candidates:
            self.candidates.append((int(can.center.x),
                                    int(can.center.y),
                                    int(can.diameter / 2.0)))
        self.latest_image_id = candidates.header.stamp
        self.new_can = True


if __name__ == "__main__":
    Classifier()
