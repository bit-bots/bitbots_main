#! /usr/bin/env python2
import numpy as np
from .color import ColorDetector
from .horizon import HorizonDetector
from .candidate import Candidate
import cv2
import math


class LineDetector2:
    def __init__(self, image, candidates, white_detector, horizon_detector, config):
        # type: (np.matrix, list, ColorDetector, HorizonDetector, dict) -> None
        self.image = image
        self.candidates = candidates
        self.white_detector = white_detector
        self.horizon_detector = horizon_detector
        self.config = config
        self._linepoints = None
        self._linesegments = None

    def set_candidates(self, candidates):
        # type: (list) -> None
        self._candidates = candidates

    def get_linepoints(self):
        if self._linepoints is None:
            if self._linesegments is None:
                self.get_linesegments()
            self._linepoints = []
            for ls in self._linesegments:
                self._linepoints.append((ls[0], ls[1]))
                self._linepoints.append((ls[2], ls[3]))

        return self._linepoints

    def get_linesegments(self):
        if self._linesegments is not None:
            return self._linesegments

        blurred = cv2.blur(self.image, (5, 5))

        # setting up vertical kernels
        bkernel = np.array([0, 0, 0,
                            0, 0, 0,
                            -1, -2, -1])
        rkernel = np.array([0, 0, 0,
                            0, 0, 0,
                            -1, -2, -1])
        gkernel = np.array([1, 2, 1,
                            0, 0, 0,
                            -1, -2, -1])
        # correlating with kernels with each channel and adding results
        bfiltered = cv2.filter2D(blurred[:, :, 0], -1, bkernel)
        gfiltered = cv2.filter2D(blurred[:, :, 1], -1, gkernel)
        rfiltered = cv2.filter2D(blurred[:, :, 2], -1, rkernel)
        filtered1 = cv2.add(cv2.add(bfiltered, gfiltered), rfiltered)

        bfiltered = cv2.filter2D(blurred[:, :, 0], -1, cv2.flip(bkernel, 0))
        gfiltered = cv2.filter2D(blurred[:, :, 1], -1, cv2.flip(gkernel, 0))
        rfiltered = cv2.filter2D(blurred[:, :, 2], -1, cv2.flip(rkernel, 0))
        filtered2 = cv2.add(cv2.add(bfiltered, gfiltered), rfiltered)

        # setting up horizontal kernels
        bkernel2 = np.array([0, 0, -1,
                             0, 0, -2,
                             0, 0, -1])
        rkernel2 = np.array([0, 0, -1,
                             0, 0, -2,
                             0, 0, -1])
        gkernel2 = np.array([1, 0, -1,
                             2, 0, -2,
                             1, 0, -1])
        # correlating with kernels with each channel and adding results
        bfiltered = cv2.filter2D(blurred[:, :, 0], -1, bkernel2)
        gfiltered = cv2.filter2D(blurred[:, :, 1], -1, gkernel2)
        rfiltered = cv2.filter2D(blurred[:, :, 2], -1, rkernel2)
        filtered3 = cv2.add(cv2.add(bfiltered, gfiltered), rfiltered)

        bfiltered = cv2.filter2D(blurred[:, :, 0], -1, cv2.flip(bkernel2, 1))
        gfiltered = cv2.filter2D(blurred[:, :, 1], -1, cv2.flip(gkernel2, 1))
        rfiltered = cv2.filter2D(blurred[:, :, 2], -1, cv2.flip(rkernel2, 1))
        filtered4 = cv2.add(cv2.add(bfiltered, gfiltered), rfiltered)

        # adding results of each convolution
        filtered = cv2.add(cv2.add(filtered1, filtered2), cv2.add(filtered3, filtered4))

        # TODO better filter for low values
        # TODO ROSPARAM
        filtered_sub = cv2.subtract(filtered, 40)
        lines = cv2.HoughLinesP(filtered_sub, 1, math.pi / 180, 10, minLineLength=10)

        self._linesegments = []
        for l in lines:
            for x1, y1, x2, y2 in l:
                # check if start or end is in any of the candidates
                in_candidate = False
                for candidate in self.candidates:
                    if self._point_in_candidate((x1, x2), candidate) or self._point_in_candidate((x2,y2), candidate):
                        in_candidate = True
                        break;
                # check if start and and is under horizon
                under_horizon = self.horizon_detector.point_under_horizon((x1, y1), 50) and \
                                self.horizon_detector.point_under_horizon((x1, y1), 50)
                if not in_candidate and under_horizon:
                    self._linesegments.append((x1, y1, x2, y2))
        cv2.imshow("filtered_sub", filtered_sub)
        cv2.waitKey(1)
        return self._linesegments

    def _point_in_candidate(self, point, candidate):
        #type: (tuple, Candidate) -> bool
        return candidate.get_upper_left_x() <= point[0] <= candidate.get_upper_left_x() + candidate.get_width() and \
               candidate.get_upper_left_y() <= point[1] <= candidate.get_upper_left_y() + candidate.get_height()
