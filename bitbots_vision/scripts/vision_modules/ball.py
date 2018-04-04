
import sys
import numpy as np
import cv2


class Ball:
    def __init__(self, x1=0, y1=0, width=0, height=0):
        self._x1 = x1
        self._y1 = y1
        self._width = width
        self._height = height

    def get_width(self):
        return self._width

    def get_height(self):
        return self._height

    def get_center_x(self):
        return self._x1 + int(self._width // 2)

    def get_center_y(self):
        self._y1 + int(self._height // 2)

    def get_center_point(self):
        return self.get_center_x(), self.get_center_y()

    def get_diameter(self):
        return int((self._height + self._width) // 2)

    def get_radius(self):
        return int(self.get_diameter() // 2)

    def get_upper_left_point(self):
        return self._x1, self._y1

    def get_upper_left_x(self):
        return self._x1

    def get_upper_left_y(self):
        return self._y1


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
                                                              scaleFactor=self._scale_factor,
                                                              minNeighbors=self._min_neighbors,
                                                              minSize=(self._min_size, self._min_size))
        return self._candidates
