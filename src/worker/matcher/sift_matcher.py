from __future__ import absolute_import

from .interface import Matcher

import cv2
import numpy as np
from silx.image import sift


class SiftMatcher(Matcher):
    def __init__(self, config):
        self.devicetype = "CPU"
        self.shape = None
        self.dtype = None
        self.siftPlan = None
        self.matchPlan = None
        self.feature_dtype = np.dtype((np.record, [('x', '<f4'), ('y', '<f4'), ('scale', '<f4'), ('angle', '<f4'), ('desc', 'u1', (128,))]))
        self.set_config(config)

    def match(self, kp, desc1, desc2):
        keypoints = list(map(lambda keypoint: ( keypoint.pt[0], 
                                                keypoint.pt[1],
                                                keypoint.size,
                                                keypoint.angle), kp))
        features1 = np.zeros((desc1.shape[0],), dtype=self.feature_dtype)
        for index, keypoint in enumerate(keypoints):
            features1[index]['angle'] = keypoints[3]
        features1['desc'] = desc1
        features2 = np.zeros((desc2.shape[0],), dtype=self.feature_dtype)
        features2['desc'] = desc2

        if self.matchPlan is None:
            self.initSift()
        try:
            matches = self.matchPlan(features1, features2)
        except(Exception):
            return []

        matches = matches[['x', 'y', 'scale','angle']][:,0].tolist()
        points = [cv2.KeyPoint(match[0], match[1], match[2], match[3]) for match in matches]
        return points

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
        
        features = self.siftPlan.keypoints(image)
        descriptors = features['desc']
        keypoints = features[['x', 'y', 'scale', 'angle']]
        keypoints = keypoints.tolist()
        keypoints = list(map(lambda keypoint: cv2.KeyPoint( float(keypoint[0]), 
                                                            float(keypoint[1]),
                                                            float(keypoint[2]),
                                                            float(keypoint[3])), keypoints))
        return keypoints, descriptors

    def debug_keypoints(self, image, debug_keypoints, color):
        return cv2.drawKeypoints(image, debug_keypoints, None, color=color, flags=0)

    def set_config(self, config):
        pass
