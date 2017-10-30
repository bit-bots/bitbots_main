import numpy as np

class HorizonDetector:

    def __init__(self, image, color_detector):
        self.image = image
        self.color_detector = color_detector
        self._horizon_points = None
        self._horizon_full = None
        self._x_steps = 30
        self._y_steps = 30

    def get_horizon_points(self):
        if self._horizon_points is None:
            y_stepsize = self.image.shape[0] / (self._y_steps-1)
            x_stepsize = self.image.shape[1] / (self._x_steps-1)
            self._horizon_points = []
            for x_step in range(self._x_steps):
                x = round(x_step * x_stepsize)
                for y_step in range(self._y_steps):

                                        pass
        return self._horizon_points

    def get_full_horizon(self):
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
