#!/usr/bin/env python2
import numpy as np
import os
import cv2
import silx
import sys
import time
from silx.image import sift

class VisualCompass():

    def __init__(self, config):
        self.setConfig(config)

    def processImage(self, image, callback=None):
        pass

    def setConfig(self, config):
        pass

    def setTruth(self, angle, image):
        pass

    def getSide(self):
        pass