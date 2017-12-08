
import sys
import numpy as np
import cv2


class BallFinder:
    def __init__(self, image, cascade, config):
        # type: (np.matrix, cv2.CascadeClassifier, dict) -> BallFinder
        self._candidates = None
        self._ball = None
        self._cascade = cascade
        self._image = image

        self._debug = False

        # init config
        self._classify_threshold = config['classify_threshold']
        self._scale_factor = config['scale_factor']
        self._min_neighbors = config['min_neighbors']
        self._min_size = config['min_size']

    def get_candidates(self):
        # type: () -> list
        if self._candidates is None:
            image_gray = cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY)
            self._candidates = self._cascade.detectMultiScale(image_gray,
                                                              scaleFactor=_scale_factor,
                                                              minNeighbors=_min_neighbors,
                                                              minSize=_min_size)
        return self._candidates
