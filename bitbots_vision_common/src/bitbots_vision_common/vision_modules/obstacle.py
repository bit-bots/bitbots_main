import numpy as np

from .candidate import CandidateFinder, Candidate
from .color import ColorDetector
from .horizon import HorizonDetector


class ObstacleDetector(CandidateFinder):
    def __init__(self, red_color_detector, blue_color_detector, horizon_detector, config):
        # type: (ColorDetector, ColorDetector, HorizonDetector, dict) -> None
        self._red_color_detector = red_color_detector
        self._blue_color_detector = blue_color_detector
        self._horizon_detector = horizon_detector
        self._color_threshold = config['color_threshold']
        self._horizon_diff_threshold = config['horizon_diff_threshold']
        self._candidate_horizon_offset = config['candidate_horizon_offset']

        self._image = None
        self._blue_mask = None
        self._red_mask = None
        self._obstacles = None
        self._blue_obstacles = None
        self._red_obstacles = None

    def set_image(self, image):
        self._image = image
        self._blue_mask = None
        self._red_mask = None
        self._obstacles = None
        self._blue_obstacles = None
        self._red_obstacles = None

    def get_top_candidates(self, count=1):
        """ This is bullshit for the abstract class"""
        return self.get_candidates()

    def get_candidates(self):
        # type: () -> list[Candidate]
        if self._obstacles is None:
            self._obstacles = list()
            obstacle_begin = None
            horizon_points = self._horizon_detector.get_horizon_points()
            a = horizon_points[0]
            b = None
            for point in horizon_points[1:]:
                b = point
                if not obstacle_begin:
                    if b[1] - a[1] > self._horizon_diff_threshold:
                        obstacle_begin = a
                else:
                    if a[1] - b[1] > self._horizon_diff_threshold:
                        self._obstacles.append(
                            Candidate(
                                obstacle_begin[0],
                                max(
                                    0,
                                    obstacle_begin[1] - self._candidate_horizon_offset),
                                b[0] - obstacle_begin[0],
                                a[1] - max(
                                           0,
                                           obstacle_begin[1] - self._candidate_horizon_offset)
                            )
                        )
                        obstacle_begin = None
                a = b
            if obstacle_begin:
                self._obstacles.append(
                    Candidate(
                        obstacle_begin[0],
                        max(
                            0,
                            obstacle_begin[1] - self._candidate_horizon_offset),
                        b[0] - obstacle_begin[0],
                        a[1] - max(
                                   0,
                                   obstacle_begin[1] - self._candidate_horizon_offset)
                    )
                )
        return self._obstacles

    def get_red_obstacles(self):
        if self._red_obstacles is None:
            self._colorsort_obstacles()
        return self._red_obstacles

    def get_blue_obstacles(self):
        if self._blue_obstacles is None:
            self._colorsort_obstacles()
        return self._blue_obstacles

    def _colorsort_obstacles(self):
        if not self._blue_mask:
            self._blue_mask = self._blue_color_detector.mask_image(self._image)
        if not self._red_mask:
            self._red_mask = self._red_color_detector.mask_image(self._image)
        self._red_obstacles = list()
        self._blue_obstacles = list()
        for obstacle in self.get_candidates():
            blueness = np.mean(
                self._blue_mask[
                    obstacle.get_upper_left_y():obstacle.get_lower_right_y(),
                    obstacle.get_upper_left_x():obstacle.get_lower_right_x()
                ]
            )
            redness = np.mean(
                self._red_mask[
                    obstacle.get_upper_left_y():obstacle.get_lower_right_y(),
                    obstacle.get_upper_left_x():obstacle.get_lower_right_x()
                ]
            )
            if redness > self._color_threshold and redness > blueness:
                self._red_obstacles.append(obstacle)
                continue
            if blueness > self._color_threshold:
                self._blue_obstacles.append(obstacle)
                continue
