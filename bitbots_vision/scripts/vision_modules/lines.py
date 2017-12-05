
import sys
from random import randint
from humanoid_league_msgs.msg import LineSegmentInImage
import math
import cv2


class Lines:
    def __init__(self, image, candidates, colordetector):
        self._image = image
        self._candidates = candidates
        self._linepoints = None
        self._debug = False
        self._candidates = candidates
        self._colordetector = colordetector


    def get_linepoints(self):

        bimg = cv2.GaussianBlur(self._image, (9, 9), 0)
       # mask = cv2.inRange(bimg, self.green_min, self.green_max)

        for x in range(1000):

            p = randint(0, bimg.shape[0] - 1), randint(0, bimg.shape[1] - 31)

            #if self.under_horizon(horizon, stepwidth, p): # todo check if under horizon
           # if mask[p[0], p[1]] == 0 and sum(bimg[p[0], p[1]]) > 400:
            if self._colordetector.match_pixel(p[0]) and self._colordetector.match_pixel(p[1]):
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
