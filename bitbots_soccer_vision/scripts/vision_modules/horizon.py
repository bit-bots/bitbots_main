class HorizonDetector:

    def __init__(self, image, color_detector):
        self.image = image
        self.color_detector = color_detector
        self._horizon_points = None
        self._horizon_full = None
        self._x_steps = 30
        self._y_steps = 30

    def get_horizon_points(self):
        y_stepsize = self.image.shape[0] / (self._y_steps-1)
        x_stepsize = self.image.shape[1] / (self._x_steps-1)
        for x in range(self._x_steps):
            for y in range(self._y_steps):

        pass

    def get_full_horizon(self):
        pass

    def point_under_horizon(self, point, offset) -> bool:
        pass

    def _interpolate_points(self, points):
        pass

    def _equalize_points(self, points):
        for index, point in enumerate(points[1:-1]):
            point_before =
