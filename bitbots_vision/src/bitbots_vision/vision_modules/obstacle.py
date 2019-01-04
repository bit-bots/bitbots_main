from .candidate import CandidateFinder, Candidate
from .color import ColorDetector
from .horizon import HorizonDetector
from .evaluator import RuntimeEvaluator
from .debug import DebugPrinter
import cv2
import numpy as np


class ObstacleDetector(CandidateFinder):
    def __init__(self, red_color_detector, blue_color_detector, white_color_detector, horizon_detector,
                 runtime_evaluator, config, debug_printer):
        # type: (ColorDetector, ColorDetector, ColorDetector, HorizonDetector, RuntimeEvaluator, dict, DebugPrinter) -> None
        self._debug_printer = debug_printer
        self._red_color_detector = red_color_detector
        self._blue_color_detector = blue_color_detector
        self._white_color_detector = white_color_detector
        self._horizon_detector = horizon_detector
        self._runtime_evaluator = runtime_evaluator
        self._color_threshold = config['obstacle_color_threshold']
        self._white_threshold = config['obstacle_white_threshold']
        self._horizon_diff_threshold = config['obstacle_horizon_diff_threshold']
        self._candidate_horizon_offset = config['obstacle_candidate_horizon_offset']
        self._candidate_min_width = config['obstacle_candidate_min_width']
        self._finder_step_length = config['obstacle_finder_step_length']

        self._image = None
        self._blue_mask = None
        self._red_mask = None
        self._white_mask = None

        self._obstacles = None
        self._blue_obstacles = None
        self._red_obstacles = None
        self._white_obstacles = None
        self._other_obstacles = None

    def set_image(self, image):
        self._image = image
        self._blue_mask = None
        self._red_mask = None
        self._white_mask = None
        self._obstacles = None
        self._blue_obstacles = None
        self._red_obstacles = None
        self._white_obstacles = None
        self._other_obstacles = None

    def get_top_candidates(self, count=1):
        """ This is bullshit for the abstract class"""
        return self.get_candidates()

    def get_candidates(self):
        """ this method can be used to switch between get_candidates_fast and get_candidates_accurate"""
        return self._get_convex_horizon_candidates()

    def _get_step_horizon_candidates(self):
        # type: () -> list[Candidate]
        """
        finds candidates by comparing the height of adjacent horizon points
        faster, less accurate alternative to get_candidates_convex
        :return: candidate(int: x upper left point, int: y upper left point, int: width, int: height)
        """
        if self._obstacles is None:
            self._runtime_evaluator.start_timer() # for runtime testing
            self._obstacles = list()
            obstacle_begin = None
            horizon_points = self._horizon_detector.get_horizon_points()
            a = horizon_points[0]  # first point of horizon_points
            b = None
            for point in horizon_points[1:]:  # traverses horizon_points from left to right
                b = point  # assigns the next point of horizon_points
                if not obstacle_begin:  # checks whether the beginning of an obstacle has already bin found
                    if b[1] - a[1] > self._horizon_diff_threshold:
                        # checks whether the horizon goes downhill by comparing the heights of b and a
                        obstacle_begin = a  # the obstacle begins at the left point of these two
                else:
                    if a[1] - b[1] > self._horizon_diff_threshold:
                        # checks whether the horizon goes uphill again by comparing the heights of a and b
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
            if obstacle_begin:  # obstacle began but never ended (problematic edge-case):
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
            self._runtime_evaluator.stop_timer()  # for runtime testing
            self._runtime_evaluator.print_timer()  # for runtime testing
        return self._obstacles

    def _get_convex_horizon_candidates(self):
        # type: () -> list[Candidate]
        """
        finds candidates using the difference of the convex horizon and the normal horizon.
        Alternative to get_candidates (more accurate, about 0.0015 seconds slower)
        :return: candidate(int: x upper left point, int: y upper left point, int: width, int: height)
        """
        # Todo: fix rarely finding not existent obstacles at the edge (vectors + orthogonal distance?)
        # Todo: increase step length before beginning of obstacle has been found, decrease it afterwards
        # Todo: interpolate individual points instead of the whole list (see get_full_horizon)
        if self._obstacles is None:
            self._runtime_evaluator.start_timer()  # for runtime testing
            self._obstacles = list()
            obstacle_begin = None
            """
            the ordinary horizon and convex_horizon consist out of a limited amount of points (usually 30).
            the full_horizon/full_convex_horizon have interpolated points
             to have as many points as the width of the picture.
            """
            full_convex_horizon = np.array(self._horizon_detector.get_full_convex_horizon()).astype(int)
            full_horizon = np.array(self._horizon_detector.get_full_horizon()).astype(int)
            """
            calculates the distance between the points of the full_horizon and full_convex_horizon
            horizon_distance is a list of distances with the index being the corresponding x-coordinate
            """
            horizon_distance = full_horizon-full_convex_horizon
            """
            threshold determines the minimum distance of the two horizons for an object to be found
            minWidth determines the minimum width of potential objects to be identified as candidates
            step is the length of one step in pixel: lager step -> faster, but more inaccurate
            """
            # Todo: value of minWidth and step has to be tested
            threshold = self._horizon_diff_threshold
            min_width = self._candidate_min_width  # minimal width of an acceptable candidate in pixels
            step = self._finder_step_length  # step size in the interpolated horizon
            pic_width = len(horizon_distance)  # Width of picture
            for i in range(0, pic_width, step):  # traverses horizon_distance
                if not obstacle_begin:
                    if horizon_distance[i] > threshold:
                        obstacle_begin = (i, full_convex_horizon[i])  # found beginning of obstacle
                else:
                    if horizon_distance[i] < threshold:  # found end of obstacle
                        # candidate(x upper left point, y upper left point, width, height)
                        x = obstacle_begin[0]
                        w = i-x
                        if w > min_width:
                            y = max(0, obstacle_begin[1] - self._candidate_horizon_offset)
                            h = np.round(full_horizon[i - w / 2] - y)
                            self._obstacles.append(Candidate(x, y, w, h))
                        obstacle_begin = None
            if obstacle_begin:
                # obstacle began but never ended (problematic edge-case):
                # candidate(x upper left point, y upper left point, width, height)
                i = pic_width  # we have to reinitialise i because it was only usable in the for-loop
                x = obstacle_begin[0]
                w = i - x
                if w > min_width:
                    y = max(0, obstacle_begin[1] - self._candidate_horizon_offset)
                    h = full_horizon[i - w / 2] - y
                    self._obstacles.append(Candidate(x, y, w, h))
            self._runtime_evaluator.stop_timer()  # for runtime testing
            self._runtime_evaluator.print_timer()  # for runtime testing
        return self._obstacles

    def get_all_obstacles(self):
        self.get_candidates()

    def get_red_obstacles(self):
        if self._red_obstacles is None:
            self._colorsort_obstacles()
        return self._red_obstacles

    def get_blue_obstacles(self):
        if self._blue_obstacles is None:
            self._colorsort_obstacles()
        return self._blue_obstacles

    def get_white_obstacles(self):
        if self._white_obstacles is None:
            self._colorsort_obstacles()
        return self._white_obstacles

    def get_other_obstacles(self):
        if self._other_obstacles is None:
            self._colorsort_obstacles()
        return self._other_obstacles

    def _colorsort_obstacles(self):
        if not self._blue_mask:
            self._blue_mask = self._blue_color_detector.mask_image(self._image)
        if not self._red_mask:
            self._red_mask = self._red_color_detector.mask_image(self._image)
        if not self._white_mask:
            self._white_mask = self._white_color_detector.mask_image(self._image)
        self._red_obstacles = list()
        self._blue_obstacles = list()
        self._white_obstacles = list()
        self._other_obstacles = list()
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
            whiteness = np.mean(
                self._white_mask[
                    obstacle.get_upper_left_y():obstacle.get_lower_right_y(),
                    obstacle.get_upper_left_x():obstacle.get_lower_right_x()
                ]
            )

            # players are the priority here
            if redness > self._color_threshold and redness > blueness:
                self._red_obstacles.append(obstacle)
                continue
            elif blueness > self._color_threshold:
                self._blue_obstacles.append(obstacle)
                continue
            elif whiteness > self._white_threshold:
                self._white_obstacles.append(obstacle)
                continue
            else:
                self._other_obstacles.append(obstacle)
