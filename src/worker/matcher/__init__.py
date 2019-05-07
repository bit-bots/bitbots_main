from __future__ import absolute_import

from .orb_matcher import OrbMatcher
from .sift_matcher import SiftMatcher
from .akaze_matcher import AkazeMatcher
from .interface import Matcher as MatcherInterface
import cv2

class Matcher (MatcherInterface):
    def __init__(self, config):
        self.matcher = None
        self.matcherType = None
        self.matcherClasses = {
            "orb": OrbMatcher,
            "sift": SiftMatcher,
            "akaze": AkazeMatcher
        }

        self.set_config(config)

    #returns number of matches
    def match(self, keypoints, descriptor1, descriptor2):
        return self.matcher.match(keypoints, descriptor1, descriptor2)

    def get_keypoints(self, image):
        return self.matcher.get_keypoints(image)

    def debug_keypoints(self, image, debug_keypoints, color):
        return self.matcher.debug_keypoints(image, debug_keypoints, color)

    def set_config(self, config):
        matcher_type = config['compass_matcher']
        if matcher_type == self.matcherType:
            self.matcher.set_config(config)
        else:
            self.matcherType = matcher_type
            if matcher_type not in self.matcherClasses:
                raise AssertionError(self.matcherType + ": Matcher not available!")
            matcher_class = self.matcherClasses[self.matcherType]
            self.matcher = matcher_class(config)