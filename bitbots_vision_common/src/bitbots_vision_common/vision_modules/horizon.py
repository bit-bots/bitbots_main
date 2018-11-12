import numpy as np
import cv2
import rospy
from .color import ColorDetector
from operator import itemgetter


class HorizonDetector:

    def __init__(self, color_detector, config):
        # type: (np.matrix, ColorDetector, dict) -> None
        self._image = None
        self._color_detector = color_detector
        self._horizon_points = None
        self._horizon_full = None
        # init config
        self._x_steps = config['x_steps']
        self._y_steps = config['y_steps']
        self._precise_pixel = config['precise_pixel']
        self._min_precise_pixel = config['min_precise_pixel']

    def set_image(self, image):
        self._image = image
        self._horizon_points = None
        self._horizon_full = None

    def get_horizon_points(self):
        # type: () -> list
        """
        calculates the horizon if not calculated yet and returns a list
        containing coordinates on the picture where the horizon is.
        :return list of x,y tuples of the horizon:
        """
        if self._horizon_points is None:
            self._horizon_points = self._sub_horizon()
        return self._horizon_points

    def _sub_horizon(self):
        mask = self._color_detector.mask_image(self._image)
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)
        mask = cv2.resize(mask, (self._y_steps, self._y_steps), interpolation=cv2.INTER_LINEAR)
        min_y = self._image.shape[0] - 1
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)
        horizon_points = []
        for x_step in range(self._x_steps):  # traverse columns
            firstgreen = min_y  # set horizon point to worst case
            x = int(round(x_step * x_stepsize))  # get x value of step (depends on image size)
            for y_step in range(self._y_steps):  # traverse rows
                y = int(round(y_step * y_stepsize))  # get y value of step (depends on image size)
                if mask[y_step, x_step] > 100:  # when the pixel is in the color space
                    firstgreen = y
                    break
            horizon_points.append((x, firstgreen))
        return horizon_points

    def _mask_horizon(self):
        mask = self._color_detector.mask_image(self._image)
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=2)
        min_y = self._image.shape[0] - 1
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)
        horizon_points = []
        for x_step in range(self._x_steps):  # traverse columns
            firstgreen = min_y  # set horizon point to worst case
            x = int(round(x_step * x_stepsize))  # get x value of step (depends on image size)
            for y_step in range(self._y_steps):  # traverse rows
                y = int(round(y_step * y_stepsize))  # get y value of step (depends on image size)
                if np.mean(mask[max(0, y-2):(y+3), max(0, x-2):(x+3)]) > 100:  # when the pixel is in the color space
                    firstgreen = y
                    break
            horizon_points.append((x, firstgreen))
        return horizon_points

    def _precise_horizon(self):
        # type: () -> list
        """
        Calculates the horizon coordinates in a precise way, but less fast and efficient.
        It checks after having found a horizon if coordinates around this point are also green
        and thus under the horizon.
        It additionally employs checking between the last point known as not the horizon and the horizon point
        to see if the horizon starts somewhere in between. (Currently actually a TODO)
        see also: _fast_horizon()
        :return list of coordinates of the horizon:
        """
        # worst case:
        min_y = self._image.shape[0] - 1
        y_stepsize = (self._image.shape[0] - 1) / float(self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / float(self._x_steps - 1)
        horizon_points = []
        for x_step in range(self._x_steps):  # traverse columns
            firstgreen = min_y  # set horizon point to worst case
            x = int(round(x_step * x_stepsize))  # get x value of step (depends on image size)
            for y_step in range(self._y_steps):  # traverse rows
                y = int(round(y_step * y_stepsize))  # get y value of step (depends on image size)
                if self._color_detector.match_pixel(self._image[y][x]):  # when the pixel is in the color space
                    if (y + self._precise_pixel) < min_y:
                        for i in range(self._precise_pixel):
                            greencount = 0
                            if self._color_detector.match_pixel(self._image[y, x]):
                                greencount += 1
                            if greencount >= self._min_precise_pixel:
                                firstgreen = y
                                break
                        firstgreen = y
                        firstgreen_precise = int(round(
                            (firstgreen - y_stepsize)
                            / 2.0))
                        if firstgreen_precise >= 0 and \
                                self._color_detector.match_pixel(
                                    self._image[firstgreen_precise, x]):
                            firstgreen = firstgreen_precise
                        break
            horizon_points.append((x, firstgreen))
        return horizon_points

    def get_full_horizon(self):
        # type: () -> list
        """
        calculates an interpolated list of y coordinates where the horizon is for the picture
        the index of the y value is the x coordinate on the picture
        :return list of y coordinates where the horizon is. Index of y value is the x coordinate:
        """
        if self._horizon_full is None:
            xp, fp = zip(*self.get_horizon_points())
            x = list(range(self._image.shape[1]))
            self._horizon_full = np.interp(x, list(xp), list(fp))
        return self._horizon_full

    def candidate_under_horizon(self, candidate, y_offset=0):
        # type: (tuple, int) -> bool
        """
        returns whether the candidate is under the horizon or not
        :param candidate: the candidate, a tuple (upleft_x, upleft_y, width, height)
        :param y_offset: an offset in y-direction (higher offset allows points in a wider range over the horizon)
        :return: whether the candidate is under the horizon or not
        """
        footpoint = (candidate[0] + candidate[2] // 2, candidate[1] + candidate[3] + y_offset)
        return self.point_under_horizon(footpoint)

    def candidates_under_horizon(self, candidates, y_offset=0):
        # type: (list, int) -> list
        return [candidate for candidate in candidates if self.candidate_under_horizon(candidate, y_offset)]

    def balls_under_horizon(self, balls, y_offset=0):
        # type: (list, int) -> list
        return [candidate for candidate in balls if self.candidate_under_horizon(
            (candidate.get_upper_left_x(),
             candidate.get_upper_left_y(),
             candidate.get_width(),
             candidate.get_height()),
            y_offset)]

    def point_under_horizon(self, point, offset=0):
        # type: (tuple, int) -> bool
        """
        returns if given coordinate is a point under horizon
        :param point: coordinate (x, y) to test
        :param offset: offset of pixels to still be accepted as under the horizon. Default is 0.
        :return a boolean if point is under horizon:
        """
        if not 0 <= point[0] < len(self.get_full_horizon()):
            rospy.logwarn('point_under_horizon got called with an out of bounds horizon point')
            return False
        return point[1] + offset > self.get_full_horizon()[point[0]]

    def get_upper_bound(self, y_offset=0):
        # type: () -> int
        """
        returns the y-value of highest point of the horizon (lowest y-value)
        :return: int(), y-value of highest point of the horizon (lowest y-value)
        """
        return max(0, int(min(self.get_horizon_points(), key=itemgetter(1))[1] - y_offset))

    def _equalize_points(self, points):
        # type: (list) -> list
        """
        returns a list of the input points with smoothed y-coordinates to reduce
        the impact of outlier points in the horizon, which are caused by
        detection errors
        :param points: list of input points consisting of tuples (x, y)
        :return: list of input points with smoothed y-coordinates consisting of tuples (x, y)
        """
        equalized_points = list()
        equalized_points.append(points[0])
        buffer0 = points[0]
        buffer1 = points[1]
        for i in range(2, len(points)):
            buffer2 = points[i]
            equalized_points.append((buffer1[0], int(round((((buffer0[1] + buffer2[1]) / 2.0) + buffer1[1]) / 2.0))))
            buffer0 = buffer1
            buffer1 = buffer2
        equalized_points.append(points[-1])
        return equalized_points
