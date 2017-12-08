
import sys
from random import randint
from humanoid_league_msgs.msg import LineSegmentInImage
from.color import ColorDetector
from.horizon import HorizonDetector
import math
import numpy as np
import cv2
import rospy


class LineDetector:
    def __init__(self, image, candidates, white_detector, horizon_detector):
        # type: (np.matrix, list, ColorDetector, HorizonDetector) -> LineDetector
        self._image = image
        self._blurred_image = None
        self._candidates = candidates
        self._linepoints = None
        self._white_detector = white_detector
        self._horizon_detector = horizon_detector
        self._horizon_offset = rospy.get_param('visionparams/line_detector/horizon_offset')

    def set_candidates(self, candidates):
        # type: (list) -> None
        self._candidates = candidates

    def get_linepoints(self):
        if self._linepoints is None:
            for x in range(rospy.get_param('visionparams/line_detector/linepoints_range')):
                # point (x, y)
                p = tuple((randint(0, self._blurred_image.shape[1] - 1),
                           randint(self._horizon_detector.get_upper_bound(self._horizon_offset),
                                   self._blurred_image.shape[0] - 1)))

                if self._horizon_detector.point_under_horizon(p, self._horizon_offset):
                    if self._white_detector.match_pixel(self._blurred_image[p[1]][p[0]]):
                        is_candidate = False
                        if self._candidates is not None:
                            for candidate in self._candidates[0, :]:
                                if self._point_in_candidate(p, candidate):
                                    is_candidate = True
                                    break
                        if is_candidate:
                            continue
                        self._linepoints.append(p)
        return self._linepoints

    def _point_in_candidate(self, point, candidate):
        return candidate[0] <= point[0] <= (candidate[0] + candidate[2]) and \
               candidate[1] <= point[1] <= (candidate[1] + candidate[3])

    def _get_blurred_image(self):
        if self._blurred_image is None:
            self._blurred_image = cv2.blur(self._image, rospy.get_param('visionparams/line_detector/blur_kernel_size'))
        return self._blurred_image
