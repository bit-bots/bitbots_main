import cv2
import bisect
import math
from binary_orb import Debug
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
        self.config = None
        self.orb = None
        self.bf = None
        self.debug = Debug()
        #config values
        self.matchDistanceScalar = 0.8
        self.sampleCount = 2
        self.maxFeatureCount = 1000

    def initOrb(self):
        if self.orb is None:
            self.orb = cv2.ORB_create(nfeatures=self.maxFeatureCount)
            self.bf = cv2.BFMatcher()

    # returns list of angle matchcount pairs
    def _compare(self, descriptors):
        return map(lambda x: (x[0], self.match(descriptors, x[1])), self.groundTruth)

    # img feature descriptors -> matchcount
    def match(self, descriptors, truthdescriptors):
        self.initOrb()
        matches = self.bf.knnMatch(descriptors, truthdescriptors, k=2)
        # Apply ratio test
        good = []
        for m, n in matches:
            if m.distance < self.matchDistanceScalar * n.distance:
                good.append(m)
        return len(good)

    #returns keypoint descriptor pair
    def _get_keypoints(self, image):
        self.initOrb()
        kp, des = self.orb.detectAndCompute(image,None)
        return (kp, des)

    def set_truth(self, angle, image):
        if 0 <= angle <= 2*math.pi:
            descriptors = self._get_keypoints(image)[1]
            bisect.insort(self.groundTruth,(angle,descriptors))

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
        keypoints = self._get_keypoints(image)
        #print(descriptors)
        matches = self._compare(keypoints[1])
        #print(matches)

        self.state = self._compute_state(matches)

        if resultCB is not None:
            resultCB(*self.state)

        if debugCB is not None:
            self.debug.print_debug_info(image,keypoints, self.state, debugCB )


    def set_config(self, config):
        self.config = config
        #add self.matchDistanceScalar config
        #add self.sampleCount config
        #add self.maxFeatureCount config

    def get_side(self):
        return self.state