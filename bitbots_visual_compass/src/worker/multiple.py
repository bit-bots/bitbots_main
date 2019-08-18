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


class MultipleCompass(VisualCompassInterface):
    def __init__(self, config):

        # ([keypoints], [descriptors])
        self.feature_map = ([], [])

        # (angle, confidence, matching_count)
        self.state = (None, None)
        self.config = config
        self.matcher = None
        self.debug = Debug()
        self.angle_tagger = AngleTagger(None)

        # config values
        self.sample_count = None
        self.mean_feature_count = None
        self.mean_feature_count_list = []
        self.mean_feature_count_seed = None
        self.init_matcher()
        self.set_config(config)

    def init_matcher(self):
        if self.matcher is None:
            self.matcher = Matcher(self.config)

    def set_truth(self, angle, image):
        self.init_matcher()
        if 0 <= angle <= 2*math.pi:
            keypoints, descriptors = self.matcher.get_keypoints(image)

            old_feature_map_length = len(self.feature_map[0])
            self.feature_map[0].extend(self.angle_tagger.tag_keypoints(angle, keypoints))
            self.feature_map[1].extend(descriptors)
            self.mean_feature_count_list.append(len(descriptors) / self.mean_feature_count_seed)
            self.mean_feature_count = statistics.mean(self.mean_feature_count_list)
        print("mean_feature_count " + str(self.mean_feature_count))

    def get_feature_map(self):
        return self.feature_map

    def get_mean_feature_count(self):
        return self.mean_feature_count

    def set_feature_map(self, feature_map):
        self.feature_map = feature_map

    def set_mean_feature_count(self, mean_feature_count):
        self.mean_feature_count = mean_feature_count

    def _compute_state(self, matching_keypoints):
        angles = list(map(lambda x: x.angle, matching_keypoints))

        length = len(angles)
        if length < 2:
            return .0, .0

        z = sum(map(lambda angle: np.exp(1j * angle), angles))
        median = np.angle(z) % (math.pi * 2)
        confidence = (float(np.abs(z)) / length)
        confidence *= 1 - math.exp(-(1. / self.mean_feature_count) * length)
        return median, confidence

    def process_image(self, image, resultCB=None, debugCB=None):
        if not self.feature_map[0]:
            return
        curr_keypoints, curr_descriptors = self.matcher.get_keypoints(image)

        angle_keypoints = self.matcher.match(self.feature_map[0], np.array(self.feature_map[1]), curr_descriptors)

        self.state = self._compute_state(angle_keypoints)

        if resultCB is not None:
            resultCB(*self.state)

        if False:
        #if debugCB is not None:
            matches = self.matcher.match(curr_keypoints, curr_descriptors, self.feature_map[1])
            image = self.matcher.debug_keypoints(image, curr_keypoints, (0,0,0))

            # TODO funktioniert nicht!!!
            for value, _ in enumerate(self.feature_map):
                hue = value/float(len(self.feature_map))
                color = colorsys.hsv_to_rgb(hue,1,255)
                image = self.matcher.debug_keypoints(image, matches[value][2], color)
            self.debug.print_debug_info(image, self.state, debugCB)

        return self.state[0], self.state[1]

    def set_config(self, config):
        self.config = config
        self.sample_count = config['compass_multiple_map_image_count']
        self.mean_feature_count_seed = float(config['compass_multiple_mean_feature_count'])
        self.matcher.set_config(config)

    def get_side(self):
        return self.state
