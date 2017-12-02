
import sys
import numpy as np
import cv2


class BallFinder:
    def __init__(self, image, cascade):
        # type: (np.matrix, cv2.CascadeClassifier) -> BallFinder
        self._candidates = None
        self._ball = None
        self._cascade = cascade
        self._image = image

        self._classify_threshold = 0.5
        self._debug = False

    def get_candidates(self):
        # type: () -> list
        if self._candidates is None:
            image_gray = cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY)
            self._candidates = self._cascade.detectMultiScale(image_gray,
                                                              1.1,
                                                              1,
                                                              minSize=(10, 10))
        return self._candidates
