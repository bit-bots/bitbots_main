
import sys
from random import randint

from humanoid_league_msgs.msg import LineSegmentInImage
import math


# moving ROS to end of path to use system/venv cv2 for Python3
if "python2.7" in sys.path[1] and "python2.7" in sys.path[2]:
    sys.path.append(sys.path.pop(1))
    sys.path.append(sys.path.pop(1))
import cv2


class Lines:
    def __init__(self, image, candidates):
        self._image = image
        self._candidates = candidates
        self._linepoints = None
        self._debug = False
        self._candidates = candidates


    def get_linepoints(self):

        bimg = cv2.GaussianBlur(self._image, (9, 9), 0)
        mask = cv2.inRange(bimg, self.green_min, self.green_max)

        for x in range(1000):

            p = randint(0, bimg.shape[0] - 1), randint(0, bimg.shape[1] - 31)

            #if self.under_horizon(horizon, stepwidth, p):
            if mask[p[0], p[1]] == 0 and sum(bimg[p[0], p[1]]) > 400:
                is_ball = False
                if self._candidates is not None:
                    for b in self._candidates[0, :]:
                        if math.sqrt((b[0] - p[1]) ** 2 + (b[1] - p[0]) ** 2) < b[2] + 15:
                            is_ball = True
                if is_ball:
                    continue

                ls = LineSegmentInImage()
                ls.start.x = p[1]
                ls.start.y = p[0]
                ls.end = ls.start

                if self.debug:
                    cv2.circle(bimg, (p[1], p[0]), 1, (0, 0, 255))
        return ls
