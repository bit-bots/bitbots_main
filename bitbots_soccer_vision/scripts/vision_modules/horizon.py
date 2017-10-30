import numpy as np
from .color import ColorDetector

class HorizonDetector:

    def __init__(self, image, color_detector: ColorDetector):
        self._image = image
        self._color_detector = color_detector
        self._horizon_points = None
        self._horizon_full = None
        self._x_steps = 30
        self._y_steps = 30
        self._precise_pixel = 5
        self._min_precise_pixel = 3

    def get_horizon_points(self) -> list:
        if self._horizon_points is None:
            self._horizon_points = self._fast_horizon()
        return self._horizon_points

    def _fast_horizon(self) -> list:
        y_stepsize = self._image.shape[0] / (self._y_steps - 1)
        x_stepsize = self._image.shape[1] / (self._x_steps - 1)
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
                            temp_horizon_y = [half_y_step, x]
                    break
            horizon_points.append((x, temp_horizon_y))
        return horizon_points

    def _precise_horizon(self):
        #worst case:
        minY = self._image.shape[0]
        y_stepsize = self._image.shape[0] / (self._y_steps - 1)
        x_stepsize = self._image.shape[1] / (self._x_steps - 1)
        horizon_points = []
        for x_step in range(self._x_steps):
            greencount = 0
            firstgreen = minY
            x = round(x_step * x_stepsize)
            for y_step in range(self._y_steps):
                y = round(y_step * y_stepsize)
                if self._color_detector.match_pixel(self._image[y, x]):
                    if((y + self._precise_pixel) < minY):
                        for i in range(self._precise_pixel):
                            if self._color_detector.match_pixel(self._image[y, x]):
                                greencount += 1
                            if greencount > self._min_precise_pixel:
                                firstgreen = y
                                break
                        greencount = 0
                        firstgreen = y
                        break
            if firstgreen < minY:
                firstgreen = minY
            horizon_points.append((x,y))
        return horizon_points



    def get_full_horizon(self) -> list:
        if self._horizon_full is None:
            xp, fp = zip(*self.get_horizon_points())
            x = list(range(len(fp)+1))
            self._horizon_full = np.interp(x, xp, fp)
        return self._horizon_full


    def point_under_horizon(self, point, offset) -> bool:
        return point[1] + offset > self.get_full_horizon()[point[0]]  # Todo: catch out of bounds points


    def _equalize_points(self, points):
        for index, point in enumerate(points[1:-1]):
            point_before = points[index]
            point_after = points[index+2]
