from interface import Matcher

import cv2
from silx.image import sift


class SiftMatcher(Matcher):
    def __init__(self, config):
        self.devicetype = None
        self.shape = None
        self.dtype = None
        self.siftPlan = None
        self.matchPlan = None
        self.set_config(config)

    def match(self, kp1, kp2):
        if self.matchPlan is None:
            self.initSift()
        return len(self.matchPlan(kp1, kp2))

    def initSift(self):
        self.siftPlan = sift.SiftPlan(self.shape, self.dtype, devicetype=self.devicetype)
        self.matchPlan = sift.MatchPlan()

    def initSiftImg(self, image):
        self.shape = image.shape
        self.dtype = image.dtype
        self.initSift()


    def get_keypoints(self, image):
        if self.siftPlan is None:
            self.initSiftImg(image)
        return self.siftPlan.keypoints(image)

    # TODO copy debugger from binary_sift.py
    def debug_keypoints(self, image):
        #return cv2.drawKeypoints(image, self.get_keypoints(image), None, color=(0, 255, 0), flags=0)
        return image

    def set_config(self, config):
        self.devicetype = config['compass']['sift']['devicetype']