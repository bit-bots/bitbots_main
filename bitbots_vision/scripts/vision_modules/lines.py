
import sys
from random import randint
from humanoid_league_msgs.msg import LineSegmentInImage
from.color import ColorDetector
from.horizon import HorizonDetector
import math
import numpy as np
import cv2


class LineDetector:
    def __init__(self, image, candidates, white_detector, horizon_detector, config):
        # type: (np.matrix, list, ColorDetector, HorizonDetector, dict) -> LineDetector
        self._image = image
        self._blurred_image = None
        self._candidates = []
        if candidates:
            self._candidates = candidates
        self._linepoints = None
        self._white_detector = white_detector
        self._horizon_detector = horizon_detector
        # init config
        self._horizon_offset = config['horizon_offset']
        self._linepoints_range = config['linepoints_range']
        self._blur_kernel_size = config['blur_kernel_size']

    def set_candidates(self, candidates):
        # type: (list) -> None
        self._candidates = candidates

    def get_linepoints(self):
        if self._linepoints is None:
            self._linepoints = list()
            for x in range(self._linepoints_range):
                # point (x, y)
                p = tuple((randint(0, self._get_blurred_image().shape[1] - 1),
                           randint(self._horizon_detector.get_upper_bound(self._horizon_offset),
                                   self._get_blurred_image().shape[0] - 1)))

                if self._horizon_detector.point_under_horizon(p, self._horizon_offset):
                    if self._white_detector.match_pixel(self._get_blurred_image()[p[1]][p[0]]):
                        is_candidate = False
                        if self._candidates is not None:
                            for candidate in self._candidates:
                                if self._point_in_candidate(p, candidate):
                                    is_candidate = True
                                    break
                        if is_candidate:
                            continue
                        self._linepoints.append(p)
        return self._linepoints

    def _point_in_candidate(self, point, candidate):
        return candidate[0] - int(candidate[2] // 2) <= point[0] <= candidate[0] + int(candidate[2] // 2) and \
               candidate[1] - int(candidate[3] // 2) <= point[1] <= candidate[1] + int(candidate[3] // 2)

    def _get_blurred_image(self):
        if self._blurred_image is None:
            self._blurred_image = cv2.blur(self._image, (self._blur_kernel_size, self._blur_kernel_size))
        return self._blurred_image
