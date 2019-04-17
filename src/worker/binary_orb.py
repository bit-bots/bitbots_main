#!/usr/bin/env python2
import numpy as np
import cv2
import math
from interface import VisualCompass


class BinaryCompassOrb(VisualCompass):
    """
    This compass only compares two sides, hence two base images.
    """

    def __init__(self, config):
        self.config = None
        self.set_config(config)
        self.groundTruth = [None, None]
        self.debug = Debug()

    def _init_sift(self, shape, dtype):
        pass

    def process_image(self, image, resultCB=None, debugCB=None):
        """
        if None in self.groundTruth:
            return"""

        keypoints = self._get_keypoints(image)
        
        match_len = self._compare(keypoints[1])
        print(match_len)

        self.state = self._compute_state(match_len)

        if resultCB is not None:
            resultCB(*self.state)

        if debugCB is not None:
            self.debug.print_debug_info(image, keypoints, self.state, debugCB)

    def set_config(self, config):
        self.config = config

    def set_truth(self, angle, image):
        if angle == 0:
            self.groundTruth[0] = self._get_keypoints(image)[1]
        elif angle == math.pi:
            self.groundTruth[1] = self._get_keypoints(image)[1]

        if not any(elem is None for elem in self.groundTruth):
            self._clean_up_ground_truth()

    def get_side(self):
        return self.state

    def _compare(self, descriptors):
        matches = map(lambda gt: self._match(descriptors, gt), self.groundTruth)
        return matches

    def _match(self, descriptors, groundTruth):
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(descriptors,groundTruth,k=2)
        # Apply ratio test
        good = []
        for m, n in matches:
            match = m
            if match.distance < 0.8*n.distance:
                good.append(match)
        return len(good)

    def _get_keypoints(self, image):
        # Initiate STAR detector
        orb = cv2.ORB_create(nfeatures=self.config['compass']['orb']['max_feature_count'])#, scoreType=cv2.ORB_FAST_SCORE)

        kp, des = orb.detectAndCompute(image,None)

        return (kp, des)

    def _compute_state(self, matches):
        angle = 0 if matches[0] > matches[1] else math.pi

        confidence = abs(matches[0] - matches[1])/(float(sum(matches) + 1))
        return angle, confidence

    def _clean_up_ground_truth(self):
        bad = []
        print(self.groundTruth[0].shape, self.groundTruth[1].shape)
        for description in self.groundTruth:
            bad_ones = []
            for other_description in self.groundTruth:
                if not np.array_equal(other_description,description):
                    bf = cv2.BFMatcher()
                    matches = bf.knnMatch(description,other_description,k=1)
                    for m in matches:
                        match = m[0]
                        if match.distance < 200:
                            bad_ones.append(match.queryIdx)
            bad.append(bad_ones)
        for index in range(len(bad)):
            self.groundTruth[index] = np.delete(self.groundTruth[index], bad[index], axis=0) 
        print(str(map(len, bad)))
        print(self.groundTruth[0].shape, self.groundTruth[1].shape)

class Debug:

    def __init__(self):
        pass

    def print_debug_info(self, image, keypoints, state, callback):
        debug_image = image.copy()

        font = cv2.FONT_HERSHEY_SIMPLEX
        bottom_left_corner_of_text = (10, 35)
        font_scale = 1
        font_color = (255, 255, 255)
        line_type = 2

        cv2.putText(debug_image, "SIDE {} | Confidence {}".format(*state),
                    bottom_left_corner_of_text,
                    font,
                    font_scale,
                    font_color,
                    line_type)

        callback(cv2.drawKeypoints(debug_image,keypoints[0],None,color=(0,255,0), flags=0))
