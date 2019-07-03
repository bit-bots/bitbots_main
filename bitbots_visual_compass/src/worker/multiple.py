from __future__ import absolute_import

import colorsys
import math
import statistics
import cv2
from profilehooks import profile
import numpy as np
from .matcher import Matcher
from .interface import VisualCompass as VisualCompassInterface
from .debug import Debug
from .angle_tagger import AngleTagger


"""class ConfidenceModell:
    
    def getCount(self, angle):
        pass
    
    def addImage(self, angle, data):
        pass
    
    def getOrdered(self):
        
    
    def getMax(self):
        pass # returns angle
   """


class MultipleCompass(VisualCompassInterface):
    def __init__(self, config):

        # ([keypoints], [descriptors])
        self.ground_truth = ([], [])

        # (angle, confidence, matching_count)
        self.state = (None, None)
        self.config = config
        self.matcher = None
        self.debug = Debug()
        self.angle_tagger = AngleTagger(None)

        # config values
        self.sample_count = None
        self.feature_scalar = None
        self.feature_scalar_seed = None

        self.init_matcher()
        self.set_config(config)

    def init_matcher(self):
        if self.matcher is None:
            self.matcher = Matcher(self.config)

    def set_truth(self, angle, image):
        self.init_matcher()
        if 0 <= angle <= 2*math.pi:
            keypoints, descriptors = self.matcher.get_keypoints(image)

            old_ground_truth_length = len(self.ground_truth[0])
            self.ground_truth[0].extend(self.angle_tagger.tag_keypoints(angle, keypoints))
            self.ground_truth[1].extend(descriptors)
            if self.feature_scalar is None:
                self.feature_scalar = len(descriptors) / self.feature_scalar_seed
            self.feature_scalar = statistics.mean([self.feature_scalar, len(descriptors) / self.feature_scalar_seed])
        print("feature_scalar " + str(self.feature_scalar))

    def get_ground_truth_features(self):
        return self.ground_truth

    def set_ground_truth_features(self, ground_truth):
        self.ground_truth = ground_truth

    def _compute_state(self, matching_keypoints):
        angles = list(map(lambda x: x.angle, matching_keypoints))

        length = len(angles)
        if length < 2:
            return .0, .0

        z = sum(map(lambda angle: np.exp(1j * angle), angles))
        median = np.angle(z) % (math.pi * 2)
        confidence = (float(np.abs(z)) / length)
        #confidence *= 1 - math.exp(-(1. / self.feature_scalar) * length)
        return median, confidence

    def process_image(self, image, resultCB=None, debugCB=None):
        if not self.ground_truth[0]:
            return
        curr_keypoints, curr_descriptors = self.matcher.get_keypoints(image)

        angle_keypoints = self.matcher.match(self.ground_truth[0], np.array(self.ground_truth[1]), curr_descriptors)

        self.state = self._compute_state(angle_keypoints)

        if resultCB is not None:
            resultCB(*self.state)

        if False:
        #if debugCB is not None:
            matches = self.matcher.match(curr_keypoints, curr_descriptors, self.ground_truth[1])
            image = self.matcher.debug_keypoints(image, curr_keypoints, (0,0,0))

            # TODO funktioniert nicht!!!
            for value, _ in enumerate(self.ground_truth):
                hue = value/float(len(self.ground_truth))
                color = colorsys.hsv_to_rgb(hue,1,255)
                image = self.matcher.debug_keypoints(image, matches[value][2], color)
            self.debug.print_debug_info(image, self.state, debugCB)

        return self.state[0], self.state[1]

    def set_config(self, config):
        self.config = config
        self.sample_count = config['compass_multiple_ground_truth_images_count']
        self.feature_scalar_seed = float(config['compass_multiple_feature_scalar'])
        self.matcher.set_config(config)

    def get_side(self):
        return self.state
