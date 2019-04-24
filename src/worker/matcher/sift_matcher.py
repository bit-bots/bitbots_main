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
        try:
            res = len(self.matchPlan(kp1, kp2))
        except(Exception):
            return 0
        return res

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

    # TODO copy debugger from deprecated_binary_sift.py
    def debug_keypoints(self, image):
        image_copy = image.copy()
        return self._plot(image_copy, self.get_keypoints(image_copy), (0, 0, 255), 2)

    def _plot(self, image, kp, color, size):
        for i in range(kp.shape[0]):
            cv2.circle(image, (kp[i].x, kp[i].y), size + int(kp[i].scale), color, thickness=2)
        return image

    def set_config(self, config):
        self.devicetype = config['compass_sift_devicetype']