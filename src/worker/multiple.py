import cv2
import bisect
import math
from matcher import OrbMatcher
from interface import VisualCompass


"""class ConfidenceModell:
    
    def getCount(self, angle):
        pass
    
    def addImage(self, angle, data):
        pass
    
    def getOrdered(self):
        
    
    def getMax(self):
        pass # returns angle
   """


class MultipleCompass(VisualCompass):
    def __init__(self, config):
        # [(angle, data)]
        self.groundTruth = []
        self.state = (None, None)
        self.config = config
        self.matcher = None
        self.debug = Debug()
        #config values
        self.sampleCount = 2
        self.maxFeatureCount = 1000

    def initMatcher(self):
        if self.matcher is None:
            self.matcher = OrbMatcher(self.config['compass']['orb'])

    # returns list of angle matchcount pairs
    def _compare(self, matchdata):
        return map(lambda x: (x[0], self.matcher.match(matchdata, x[1])), self.groundTruth)


    def set_truth(self, angle, image):
        self.initMatcher()
        if 0 <= angle <= 2*math.pi:
            matchdata = self.matcher.get_keypoints(image)
            bisect.insort(self.groundTruth,(angle, matchdata))

    #TODO
    def _compute_state(self, matches):
        mymax = max(matches, key=lambda x: x[1])
        angle = mymax[0]
        confidence = float(mymax[1]) / self.maxFeatureCount
        return (angle, confidence)

    #TODO
    def process_image(self, image, resultCB=None, debugCB=None):
        if not self.groundTruth:
            return
        matchdata = self.matcher.get_keypoints(image)
        #print(descriptors)
        matches = self._compare(matchdata)
        #print(matches)

        self.state = self._compute_state(matches)

        if resultCB is not None:
            resultCB(*self.state)

        if debugCB is not None:
            image = self.matcher.debug_keypoints(image)
            self.debug.print_debug_info(image, self.state, debugCB)


    def set_config(self, config):
        self.config = config
        #add self.matchDistanceScalar config
        #add self.sampleCount config
        #add self.maxFeatureCount config

    def get_side(self):
        return self.state


class Debug:

    def __init__(self):
        pass

    def print_debug_info(self, image, state, callback):
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

        callback(image)