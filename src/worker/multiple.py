from debug import Debug
import bisect
import math
from matcher import Matcher
from interface import VisualCompass as VisualCompassInterface


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
        # [(angle, data)]
        self.groundTruth = []
        self.state = (None, None)
        self.config = config
        self.matcher = None
        self.debug = Debug()

        # config values
        self.sampleCount = None

        self.init_matcher()
        self.set_config(config)

    def init_matcher(self):
        if self.matcher is None:
            self.matcher = Matcher(self.config)

    # returns list of angle matchcount pairs
    def _compare(self, matchdata):
        return map(lambda x: (x[0], self.matcher.match(matchdata, x[1])), self.groundTruth)


    def set_truth(self, angle, image):
        self.init_matcher()
        if 0 <= angle <= 2*math.pi:
            matchdata = self.matcher.get_keypoints(image)
            bisect.insort(self.groundTruth,(angle, matchdata))

    #TODO
    def _compute_state(self, matches):
        mymax = max(matches, key=lambda x: x[1])
        mysum = sum(map(lambda x: x[1], matches))
        mymean = (mysum - mymax[1]) / float(max(len(matches) - 1, 1))
        angle = mymax[0]
        confidence = (mymax[1] - mymean) / (mysum + 1)
        return angle, confidence

    def process_image(self, image, resultCB=None, debugCB=None):
        if not self.groundTruth:
            return
        matchdata = self.matcher.get_keypoints(image)
        matches = self._compare(matchdata)

        self.state = self._compute_state(matches)

        if resultCB is not None:
            resultCB(*self.state)

        if debugCB is not None:
            image = self.matcher.debug_keypoints(image)
            self.debug.print_debug_info(image, self.state, debugCB)

    def set_config(self, config):
        self.config = config
        self.sampleCount = config['compass_multiple_sample_count']
        self.matcher.set_config(config)

    def get_side(self):
        return self.state