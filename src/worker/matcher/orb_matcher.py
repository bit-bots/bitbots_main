#!/usr/bin/env python2

import cv2
from interface import Matcher

class OrbMatcher(Matcher):

    def __init__(self, config):
        self.orb = None
        self.bf = cv2.BFMatcher()
        self.set_config(config)

    def get_keypoints(self, image):
        kp, des = self.orb.detectAndCompute(image,None)
        return des

    def match(self, desc1, desc2):
        matches = self.bf.knnMatch(desc1, desc2, k=2)
        good = []
        for m, n in matches:
            if m.distance < self.matchDistanceScalar * n.distance:
                good.append(m)
        return len(good)

    def set_config(self, config):
        self.matchDistanceScalar = config['compass_orb_match_distance_scalar']
        self.maxFeatureCount = config['compass_orb_max_feature_count']
        # Sets a new instance to allow changes after initialisation
        self.orb = cv2.ORB_create(nfeatures=self.maxFeatureCount)

    def debug_keypoints(self,  image):
        kp, _ = self.orb.detectAndCompute(image,None)
        return cv2.drawKeypoints(image, kp, None, color=(0, 255, 0), flags=0)