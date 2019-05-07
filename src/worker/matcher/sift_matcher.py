from __future__ import absolute_import

from .interface import Matcher

import cv2
import numpy as np
from silx.image import sift


class SiftMatcher(Matcher):
    def __init__(self, config):
        self.devicetype = None
        self.shape = None
        self.dtype = None
        self.siftPlan = None
        self.matchPlan = None
        self.set_config(config)

    def match(self, _, kp1, kp2):
        if self.matchPlan is None:
            self.initSift()
        try:
            res = self.matchPlan(kp1, kp2)
        except(Exception):
            return []
        return res

    def initSift(self):
        print(self.shape, self.dtype, self.devicetype)
        self.siftPlan = sift.SiftPlan(self.shape, self.dtype, devicetype=self.devicetype)
        self.matchPlan = sift.MatchPlan()

    def initSiftImg(self, image):
        self.shape = image.shape
        self.dtype = image.dtype
        self.initSift()

    def get_keypoints(self, image):
        if self.siftPlan is None:
            self.initSiftImg(image)
        
        keypoints = self.siftPlan.keypoints(image)
        return keypoints, keypoints

    # TODO copy debugger from deprecated_binary_sift.py
    def debug_keypoints(self, image, debug_keypoints, color):
        try:
            output_image = self._plot(image, debug_keypoints, color, 2)
        except(Exception):
            try:
                debug_keypoints_formated = self._convert_keypoint(debug_keypoints[:, 0])
                output_image = self._plot(image, debug_keypoints_formated, color, 2)
            except(Exception):
                print("Debug failed")
                output_image = image
        return output_image
        

    def _plot(self, image, kp, color, size):
        for i in range(kp.shape[0]):
            cv2.circle(image, (kp[i].x, kp[i].y), size + int(kp[i].scale), color, thickness=2)
        return image

    def _convert_keypoint(self, kps):
        d = np.dtype((np.record, [('x', '<f4'), ('y', '<f4'), ('scale', '<f4'), ('angle', '<f4'), ('desc', 'u1', (128,))]))
        return kps.astype(d)

    def set_config(self, config):
        self.devicetype = config['compass_sift_devicetype']