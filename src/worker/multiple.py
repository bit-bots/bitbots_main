from __future__ import absolute_import

import colorsys
import math
import statistics
import cv2
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
        self.state = (None, None)
        self.config = config
        self.matcher = None
        self.debug = Debug()
        self.angle_tagger = AngleTagger(None)

        # config values
        self.sampleCount = None

        self.init_matcher()
        self.set_config(config)

    def init_matcher(self):
        if self.matcher is None:
            self.matcher = Matcher(self.config)

    def set_truth(self, angle, image):
        self.init_matcher()
        if 0 <= angle <= 2*math.pi:
            keypoints, descriptors = self.matcher.get_keypoints(image)

            self.ground_truth[0].extend(self.angle_tagger.tag_keypoints(angle, keypoints))
            self.ground_truth[1].extend(descriptors)

    def get_ground_truth_keypoints(self):
        return self.ground_truth

    def set_ground_truth_keypoints(self, ground_truth):
        self.ground_truth = ground_truth

    def _compute_state(self, matching_keypoints):
        angles = list(map(lambda x: x.angle, matching_keypoints))
        angles.sort()
        length = len(angles)
        if length < 2:
            return .0, .0
        median = statistics.median(angles)
        confidence = statistics.stdev(angles) / math.pi
        confidence = confidence ** (1./5)
        return median, confidence

    def process_image(self, image, resultCB=None, debugCB=None):
        if not self.ground_truth[0]:
            return
        curr_keypoints, curr_descriptors = self.matcher.get_keypoints(image)
        #matches = self._compare(keypoints, descriptors)

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
        self.sampleCount = config['compass_multiple_sample_count']
        self.matcher.set_config(config)

    def get_side(self):
        return self.state
