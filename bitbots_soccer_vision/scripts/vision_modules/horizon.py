import numpy as np
import cv2
from .color import ColorDetector


class HorizonDetector:

    def __init__(self, image: np.matrix, color_detector: ColorDetector):
        self._image = image
        self._color_detector = color_detector
        self._horizon_points = None
        self._horizon_full = None
        self._x_steps = 30
        self._y_steps = 30
        self._precise_pixel = 5
        self._min_precise_pixel = 3

    def get_horizon_points(self) -> list:
        """
        calculates the horizon if not calculated yet and returns a list containing coordinates on the picture where the horizon is.
        :return list of x,y tuples of the horizon:
        """
        if self._horizon_points is None:
            self._horizon_points = self._equalize_points(self._precise_horizon())
        return self._horizon_points

    def _fast_horizon(self) -> list:
        """
        calculates the horizon coordinates in a quick and efficient, but less precise way.
        It calculates the horizon by checking for green and after having found the first green it also checks
        between the last point which wasn't green and the current one to see if the horizon is already inbetween.
        see also: _precise_horizon()
        :return list of coordinates of the horizon:
        """
        y_stepsize = (self._image.shape[0] - 1) / (self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / (self._x_steps - 1)
        horizon_points = []
        for x_step in range(self._x_steps):
            x = round(x_step * x_stepsize)
            temp_horizon_y = round(self._y_steps * y_stepsize)
            for y_step in range(self._y_steps):
                y = round(y_step * y_stepsize)
                # y,x because opencv image format
                if self._color_detector.match_pixel(self._image[y, x]):
                    temp_horizon_y = y
                    # going back half the step size if horizon was found
                    # doubles the precision at almost no cost
                    if y_step > 0:
                        half_y_step = round(y - (0.5 * y_stepsize))
                        if self._color_detector.match_pixel(self._image[half_y_step, x]):
                            temp_horizon_y = round(half_y_step)
                    break
            horizon_points.append((x, temp_horizon_y))
        return horizon_points

    def _precise_horizon(self):
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
        y_stepsize = (self._image.shape[0] - 1) / (self._y_steps - 1)
        x_stepsize = (self._image.shape[1] - 1) / (self._x_steps - 1)
        horizon_points = []
        for x_step in range(self._x_steps):  # traverse columns
            firstgreen = min_y  # set horizon point to worst case
            x = round(x_step * x_stepsize)  # get x value of step (depends on image size)
            for y_step in range(self._y_steps):  # traverse rows
                y = round(y_step * y_stepsize)  # get y value of step (depends on image size)
                if self._color_detector.match_pixel(self._image[y, x]):  # when the pixel is in the color space
                    if (y + self._precise_pixel) < min_y:
                        for i in range(self._precise_pixel):
                            greencount = 0
                            if self._color_detector.match_pixel(self._image[y, x]):
                                greencount += 1
                            if greencount >= self._min_precise_pixel:
                                firstgreen = y
                                break
                        firstgreen = y
                        break
            horizon_points.append((x, firstgreen))
        return horizon_points

    def get_full_horizon(self) -> list:
        """
        calculates an interpolated list of y coordinates where the horizon is for the picture
        the index of the y value is the x coordinate on the picture
        :return list of y coordinates where the horizon is. Index of y value is the x coordinate:
        """
        if self._horizon_full is None:
            xp, fp = zip(*self.get_horizon_points())
            x = list(range(len(fp)+1))
            self._horizon_full = np.interp(x, list(xp), list(fp))
        return self._horizon_full

    def candidate_under_horizon(self, candidate, y_offset):
        footpoint = (candidate[0] + candidate[2] // 2, candidate[1] + candidate[3] + y_offset)
        return self.point_under_horizon(footpoint)

    def point_under_horizon(self, point, offset=0) -> bool:
        """
        returns if given coordinate is a point under horizon
        :param point coordinate to test:
        :param offset offset of pixels to still be accepted as under the horizon. Default is 0.:
        :return a boolean if point is under horizon:
        """
        return point[1] + offset > self.get_full_horizon()[point[0]]  # Todo: catch out of bounds points

    def _equalize_points(self, points: list) -> list:
        equalized_points = list()
        equalized_points.append(points[0])
        buffer0 = points[0]
        buffer1 = points[1]
        for i in range(2, len(points)):
            buffer2 = points[i]
            equalized_points.append((buffer1[0], round((((buffer0[1] + buffer2[1]) / 2) + buffer1[1]) / 2)))
            buffer0 = buffer1
            buffer1 = buffer2
        equalized_points.append(points[-1])
        print(equalized_points)
        return equalized_points
