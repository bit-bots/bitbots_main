#!/usr/bin/env python2

from interface import VisualCompass as VisualCompassInterface
from binary_orb import BinaryCompassOrb
from binary_sift import BinaryCompassSift
from multiple import MultipleCompass


class VisualCompass(VisualCompassInterface):
    def __init__(self, config):
        self.compass = None
        self.compassType = None
        self.compassClasses = {
            "binary": BinaryCompassOrb,
            "binary_sift": BinaryCompassSift,
            "multiple": MultipleCompass
        }

        self.set_config(config)

    def process_image(self, image, resultCB=None, debugCB=None):
        return self.compass.process_image(self, image, resultCB=resultCB, debugCB=debugCB)

    def set_config(self, config):
        compass_type = config['compass_type']
        if compass_type == self.compassType:
            self.compass.set_config(config)
        else:
            self.compassType = compass_type
            if compass_type not in self.compassClasses:
                raise AssertionError(self.compassType + ": Compass not available!")
            compass_class = self.compassClasses[self.compassType]
            self.compass = compass_class(config)

    def set_truth(self, angle, image):
        return self.compass.set_truth(angle, image)

    def get_side(self):
        return self.compass.get_side()




