#!/usr/bin/env python2

from __future__ import absolute_import

import cv2
from .interface import Matcher

class OrbMatcher(Matcher):

    def __init__(self, config):
        self.orb = None
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.set_config(config)

    def get_keypoints(self, image):
        return self.orb.detectAndCompute(image,None)

    def match(self, kp, desc1, desc2):
        matches = self.bf.knnMatch(desc1, desc2, k=2)
        good = []
        for m, n in matches:
            if m.distance < self.matchDistanceScalar * n.distance:
                good.append(m.queryIdx)
        
        matched_keypoints = [kp[index] for index in good]

        return matched_keypoints

    def set_config(self, config):
        self.matchDistanceScalar = config['compass_orb_match_distance_scalar']
        self.maxFeatureCount = config['compass_orb_max_feature_count']
        # Sets a new instance to allow changes after initialisation
        self.orb = cv2.ORB_create(nfeatures=self.maxFeatureCount)

    def debug_keypoints(self,  image, debug_keypoints, color):
        return cv2.drawKeypoints(image, debug_keypoints, None, color=color, flags=0)