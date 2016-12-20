#!/usr/bin/env python2.7
from collections import OrderedDict
import cv2
import numpy as np
from keras.models import model_from_json
import rospy
from humanoid_league_msgs.msg import BallInImage, BallsInImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Classifier:
    def __init__(self):
        self.pub_ball = rospy.Publisher("/ball_in_image", BallInImage, queue_size=1)
        self.pub_rated_ball = rospy.Publisher("/rated_balls_in_image", BallsInImage, queue_size=1)

        self.bridge = CvBridge()
        self.last_imgs = OrderedDict()
        self.new_can = False
        self.latest_image_id = None

        with open("src/bitbots_ballclassifier/src/bitbots_ballclassifier/model.json", "r") as j:
            nem = model_from_json(j.read())
        nem.load_weights("src/bitbots_ballclassifier/src/bitbots_ballclassifier/model.ker")
        self.model = nem

        rospy.init_node("bitbots_ball_classifier")
        rospy.Subscriber("/usb_cam/image_raw", Image, self._image_callback, queue_size=1)
        rospy.Subscriber("/ball_candidates", BallsInImage, self._candidates_callback, queue_size=1)

        while not rospy.is_shutdown():
            if self.new_can and self.latest_image_id is not None:
                self.work()
                self.new_can = False
            else:
                rospy.sleep(0.01)

    def work(self):
        img = self.last_imgs.pop(self.latest_image_id, None)
        if img is None:
            print(self.latest_image_id + " was late")
            return
        ra = self.bridge.imgmsg_to_cv2(img, "bgr8")

        if self.candidates is not None:
            quality = []
            for i in self.candidates:
                try:
                    corp = ra[i[1] - i[2]:i[1] + i[2], i[0] - i[2]:i[0] + i[2]]

                    corp = cv2.resize(corp, (30, 30), interpolation=cv2.INTER_CUBIC)
                    corp.reshape((1,) + corp.shape)

                    p = self.model.predict(np.array([corp]), verbose=0)

                    msg = BallInImage()
                    msg.center.x = i[0]
                    msg.center.y = i[1]
                    msg.diameter = i[2] * 2
                    msg.confidence = p[0][0]
                    msg.header.frame_id = img.header.frame_id
                    quality.append(msg)
                    print(p[0][0])

                except cv2.error:
                    pass

            if len(quality) > 0:
                self.pub_ball.publish(max(quality, key=lambda x: x.confidence))
                rb = BallsInImage()
                rb.candidates = quality
                self.pub_rated_ball.publish(rb)
            else:
                self.pub_ball.publish(BallInImage())
                self.pub_rated_ball.publish(BallsInImage())

    def _image_callback(self, img):
        self.last_imgs[img.header.frame_id] = img
        if len(self.last_imgs) > 10:
            self.last_imgs.popitem(last=False)

    def _candidates_callback(self, candidates):
        self.candidates = []
        for can in candidates.candidates:
            self.candidates.append((int(can.center.x),
                                    int(can.center.y),
                                    int(can.diameter / 2.0)))
            self.latest_image_id = can.header.frame_id
        if len(candidates.candidates) == 0:
            self.latest_image_id = None
        self.new_can = True


if __name__ == "__main__":
    Classifier()

