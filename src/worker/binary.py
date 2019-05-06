import math
import colorsys 
from matcher import Matcher
from interface import VisualCompass as VisualCompassInterface
from debug import Debug


class BinaryCompass(VisualCompassInterface):
    def __init__(self, config):
        self.config = config
        self.matcher = None
        self.groundTruth = [None, None]
        self.state = (None, None)
        self.debug = Debug()

        self.init_matcher()

    def init_matcher(self):
        if self.matcher is None:
            self.matcher = Matcher(self.config)

    def process_image(self, image, resultCB=None, debugCB=None):
        keypoints, descriptor = self.matcher.get_keypoints(image)
        matches = map(lambda gt: self.matcher.match(keypoints, descriptor, gt), self.groundTruth)

        match_len = map(lambda match: len(match), matches)

        self.state = self._compute_state(match_len)

        if resultCB is not None:
            resultCB(*self.state)

        if debugCB is not None:
            image = self.matcher.debug_keypoints(image, keypoints, (0,0,0))
            for value, _ in enumerate(self.groundTruth):
                hue = value/float(len(self.groundTruth))
                color = colorsys.hsv_to_rgb(hue,1,255)
                image = self.matcher.debug_keypoints(image, matches[value], color)
            self.debug.print_debug_info(image, self.state, debugCB)

        return self.state[0], self.state[1]

    def set_truth(self, angle, image):
        if angle == 0:
            self.groundTruth[0] = self.matcher.get_keypoints(image)[1]
        elif angle == math.pi:
            self.groundTruth[1] = self.matcher.get_keypoints(image)[1]

    def set_config(self, config):
        self.config = config
        self.matcher.set_config(config)

    def _compute_state(self, matches):
        angle = 0 if matches[0] > matches[1] else math.pi

        confidence = (abs(matches[0] - matches[1])/(float(sum(matches) + 1)))**(1/float(3)) 
        return angle, confidence

    def get_side(self):
        return self.state
