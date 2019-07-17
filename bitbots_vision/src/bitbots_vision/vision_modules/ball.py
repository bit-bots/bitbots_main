
from .candidate import Candidate
import rospy
import numpy as np
import cv2


class BallFinder():
    def __init__(self, cascade, config):
        # type: (np.matrix, cv2.CascadeClassifier, dict) -> None
        self._candidates = None
        self._ball = None
        self._cascade = cascade
        self._image = None

        self._debug = False

        # init config
        self._classify_threshold = config['ball_finder_classify_threshold']
        self._scale_factor = config['ball_finder_scale_factor']
        self._min_neighbors = config['ball_finder_min_neighbors']
        self._min_size = config['ball_finder_min_size']

    def set_image(self, image):
        self._image = image
        self._candidates = None
        self._ball = None

    def set_config(self, config):

        self._classify_threshold = config['classify_threshold']
        self._scale_factor = config['scale_factor']
        self._min_neighbors = config['min_neighbors']
        self._min_size = config['min_size']


    def get_ball_candidates(self):
        # type: () -> list
        if self._candidates is None:
            image_gray = cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY)
            self._raw_candidates = self._cascade.detectMultiScale(image_gray,
                                                              scaleFactor=self._scale_factor,
                                                              minNeighbors=self._min_neighbors,
                                                              minSize=(self._min_size, self._min_size))
            self._candidates = [Candidate(*candidate) for candidate in self._raw_candidates]
        return self._candidates
