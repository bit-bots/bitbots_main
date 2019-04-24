from orb_matcher import OrbMatcher
from sift_matcher import SiftMatcher
from interface import Matcher as MatcherInterface

class Matcher (MatcherInterface):
    def __init__(self, config):
        self.matcher = None
        self.matcherType = None
        self.matcherClasses = {
            "orb": OrbMatcher,
            "sift": SiftMatcher
        }

        self.set_config(config)



    #returns number of matches
    def match(self, kp1, kp2):
        return self.matcher.match(kp1, kp2)

    def get_keypoints(self, image):
        return self.matcher.get_keypoints(image)

    def debug_keypoints(self, image):
        return self.matcher.debug_keypoints(image)

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