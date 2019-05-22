from .candidate import CandidateFinder, Candidate
from .color import ColorDetector
from .field_boundary import FieldBoundaryDetector
from .evaluator import RuntimeEvaluator
from .debug import DebugPrinter
import cv2
import numpy as np
import rospy


class ObstacleDetector(CandidateFinder):
    def __init__(self, red_color_detector, blue_color_detector, white_color_detector, field_boundary_detector,
                 runtime_evaluator, config, debug_printer):
        # type: (ColorDetector, ColorDetector, ColorDetector, FieldBoundaryDetector, RuntimeEvaluator, dict, DebugPrinter) -> None
        self._debug_printer = debug_printer
        self._red_color_detector = red_color_detector
        self._blue_color_detector = blue_color_detector
        self._white_color_detector = white_color_detector
        self._field_boundary_detector = field_boundary_detector
        self._runtime_evaluator = runtime_evaluator
        self._color_threshold = config['obstacle_color_threshold']
        self._white_threshold = config['obstacle_white_threshold']
        self._field_boundary_diff_threshold = config['obstacle_field_boundary_diff_threshold']
        self._candidate_field_boundary_offset = config['obstacle_candidate_field_boundary_offset']
        self._candidate_min_width = config['obstacle_candidate_min_width']
        self._candidate_max_width = config['obstacle_candidate_max_width']
        self._finder_step_length = config['obstacle_finder_step_length']
        self._obstacle_finder_method = config['obstacle_finder_method']
        self._distance_value_increase = config['obstacle_finder_value_increase']

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

        if self._obstacles is None:
            if self._obstacle_finder_method == 'distance':
                self._obstacles = self._obstacle_detector_distance()
            elif self._obstacle_finder_method == 'convex':
                self._obstacles = self._obstacle_detector_convex()
            else:
                self._obstacles = self._obstacle_detector_step()

        return self._obstacles


    def _obstacle_detector_step(self):
        # type: () -> list[Candidate]
        """
        finds candidates by comparing the height of adjacent field_boundary points
        faster, less accurate alternative to get_candidates_convex
        :return: candidate(int: x upper left point, int: y upper left point, int: width, int: height)
        """
        if self._obstacles is None:
            # self._runtime_evaluator.start_timer() # for runtime testing
            self._obstacles = list()
            obstacle_begin = None
            field_boundary_points = self._field_boundary_detector.get_field_boundary_points()
            a = field_boundary_points[0]  # first point of field_boundary_points
            b = None
            for point in field_boundary_points[1:]:  # traverses field_boundary_points from left to right
                b = point  # assigns the next point of field_boundary_points
                if not obstacle_begin:  # checks whether the beginning of an obstacle has already bin found
                    if b[1] - a[1] > self._field_boundary_diff_threshold:
                        # checks whether the field_boundary goes downhill by comparing the heights of b and a
                        obstacle_begin = a  # the obstacle begins at the left point of these two
                else:
                    if a[1] - b[1] > self._field_boundary_diff_threshold:
                        # checks whether the field_boundary goes uphill again by comparing the heights of a and b
                        self._obstacles.append(
                            Candidate(
                                obstacle_begin[0],
                                max(
                                    0,
                                    obstacle_begin[1] - self._candidate_field_boundary_offset),
                                b[0] - obstacle_begin[0],
                                a[1] - max(
                                           0,
                                           obstacle_begin[1] - self._candidate_field_boundary_offset)
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
                            obstacle_begin[1] - self._candidate_field_boundary_offset),
                        b[0] - obstacle_begin[0],
                        a[1] - max(
                                   0,
                                   obstacle_begin[1] - self._candidate_field_boundary_offset)
                    )
                )
            # self._runtime_evaluator.stop_timer()  # for runtime testing
            # self._runtime_evaluator.print_timer()  # for runtime testing
        return self._obstacles

    def _obstacle_detector_convex(self):
        # type: () -> list[Candidate]
        """
        finds candidates using the difference of the convex field_boundary and the normal field_boundary.
        Alternative to get_candidates (more accurate, about 0.0015 seconds slower)
        :return: candidate(int: x upper left point, int: y upper left point, int: width, int: height)
        """
        # Todo: fix rarely finding not existent obstacles at the edge (vectors + orthogonal distance?)
        # Todo: increase step length before beginning of obstacle has been found, decrease it afterwards
        # Todo: interpolate individual points instead of the whole list (see get_full_field_boundary)
        if self._obstacles is None:
            # self._runtime_evaluator.start_timer()  # for runtime testing
            self._obstacles = list()
            obstacle_begin = None
            # the ordinary field_boundary and convex_field_boundary consist out of a limited amount of points (usually 30).
            # the full_field_boundary/full_convex_field_boundary have interpolated points
            # to have as many points as the width of the picture.
            full_convex_field_boundary = np.array(self._field_boundary_detector.get_full_convex_field_boundary()).astype(int)
            full_field_boundary = np.array(self._field_boundary_detector.get_full_field_boundary()).astype(int)
            # calculates the distance between the points of the full_field_boundary and full_convex_field_boundary
            # field_boundary_distance is a list of distances with the index being the corresponding x-coordinate
            field_boundary_distance = full_field_boundary-full_convex_field_boundary
            # threshold determines the minimum distance of the two field_boundarys for an object to be found
            # minWidth determines the minimum width of potential objects to be identified as candidates
            # step is the length of one step in pixel: lager step -> faster, but more inaccurate
            # Todo: value of minWidth and step has to be tested
            threshold = self._field_boundary_diff_threshold
            min_width = self._candidate_min_width  # minimal width of an acceptable candidate in pixels
            step = self._finder_step_length  # step size in the interpolated field_boundary
            pic_width = len(field_boundary_distance)  # Width of picture
            for i in range(0, pic_width, step):  # traverses field_boundary_distance
                if not obstacle_begin:
                    if field_boundary_distance[i] > threshold:
                        obstacle_begin = (i, full_convex_field_boundary[i])  # found beginning of obstacle
                else:
                    if field_boundary_distance[i] < threshold:  # found end of obstacle
                        # candidate(x upper left point, y upper left point, width, height)
                        x = obstacle_begin[0]
                        w = i-x
                        if w > min_width:
                            y = max(0, obstacle_begin[1] - self._candidate_field_boundary_offset)
                            h = np.round(np.max(full_field_boundary[x:i]) - y)
                            if h < 0:
                                self._debug_printer.error('negative obstacle height', 'ObstacleDetection')
                            self._obstacles.append(Candidate(x, y, w, h))
                        obstacle_begin = None
            if obstacle_begin:
                # obstacle began but never ended (problematic edge-case):
                # candidate(x upper left point, y upper left point, width, height)
                i = pic_width  # we have to reinitialise i because it was only usable in the for-loop
                x = obstacle_begin[0]
                w = i - x  # calculating width of the object
                if w > min_width:  # when the width is larger than the threshold
                    y = max(0, obstacle_begin[1] - self._candidate_field_boundary_offset)  # top
                    h = np.round(np.max(full_field_boundary[x:i]) - y)
                    if h < 0:
                        self._debug_printer.error('negative obstacle height', 'ObstacleDetection')
                    self._obstacles.append(Candidate(x, y, w, h))
            # self._runtime_evaluator.stop_timer()  # for runtime testing
            # self._runtime_evaluator.print_timer()  # for runtime testing
        return self._obstacles

    def _obstacle_detector_distance(self):  # TODO: better name than 'distance'
        # type: () -> list[Candidate]
        """
        finds candidates using the difference of the convex field_boundary and the normal field_boundary.
        Detection of obstacles depends on their height in image and therefore their distance.
        :return: candidate(int: x upper left point, int: y upper left point, int: width, int: height)
        """
        # self._runtime_evaluator.start_timer()  # for runtime testing
        self._obstacles = list()
        obstacle_begin = None

        full_convex_field_boundary = np.array(
            self._field_boundary_detector.
            get_full_convex_field_boundary()).astype(int)
        full_field_boundary = np.array(self._field_boundary_detector.get_full_field_boundary()).astype(int)

        # Todo: value of minWidth and step has to be tested
        start_threshold = self._field_boundary_diff_threshold
        start_min_width = self._candidate_min_width  # minimal width of an acceptable candidate in pixels
        start_max_width = self._candidate_max_width
        distance_value_increase = float(self._distance_value_increase) / 1000
        step = self._finder_step_length  # step size in the interpolated field_boundary
        pic_width = len(full_convex_field_boundary)  # Width the image
        for i in range(0, pic_width, step):  # traverses field_boundary_distance
            current_threshold = start_threshold + int(full_field_boundary[i] * distance_value_increase)
            if not obstacle_begin:
                if (full_field_boundary[i] - full_convex_field_boundary[i]) > current_threshold:
                    obstacle_begin = (i, full_convex_field_boundary[i])  # found beginning of a potential obstacle
            else:
                if (full_field_boundary[i] - full_convex_field_boundary[i]) < current_threshold:
                    # candidate(x upper left point, y upper left point, width, height)
                    x = obstacle_begin[0]
                    w = i - x
                    y = max(0, obstacle_begin[1] - self._candidate_field_boundary_offset)
                    h = np.round(np.max(full_field_boundary[x:i]) - y)
                    current_min_width = start_min_width + int((full_convex_field_boundary[i] - h) * distance_value_increase)
                    current_max_width = start_max_width + int((full_convex_field_boundary[i] - h) * distance_value_increase)
                    if current_min_width < w < current_max_width:
                        if h < 0:
                            self._debug_printer.error('negative obstacle height', 'ObstacleDetection')
                        self._obstacles.append(Candidate(x, y, w, h))
                    obstacle_begin = None
        if obstacle_begin:
            # obstacle began but never ended (problematic edge-case):
            # candidate(x upper left point, y upper left point, width, height)
            i = pic_width-step  # we have to reinitialise i because it was only usable in the for-loop
            x = obstacle_begin[0]
            w = i - x  # calculating width of the object
            y = max(0, obstacle_begin[1] - self._candidate_field_boundary_offset)  # top
            h = np.round(np.max(full_field_boundary[x:i]) - y)
            current_min_width = start_min_width + int((full_convex_field_boundary[i] - h) * distance_value_increase)
            current_max_width = start_max_width + int((full_convex_field_boundary[i] - h) * distance_value_increase)
            if current_min_width < w < current_max_width:
                if h < 0:
                    self._debug_printer.error('negative obstacle height', 'ObstacleDetection')
                self._obstacles.append(Candidate(x, y, w, h))

        # self._runtime_evaluator.stop_timer()  # for runtime testing
        # self._runtime_evaluator.print_timer()  # for runtime testing
        return self._obstacles

    def get_all_obstacles(self):
        # type: () -> list[Candidate]
        return self.get_candidates()

    def get_red_obstacles(self):
        # type: () -> list[Candidate]
        if self._red_obstacles is None:
            self._colorsort_obstacles()
        return self._red_obstacles

    def get_blue_obstacles(self):
        # type: () -> list[Candidate]
        if self._blue_obstacles is None:
            self._colorsort_obstacles()
        return self._blue_obstacles

    def get_white_obstacles(self):
        # type: () -> list[Candidate]
        if self._white_obstacles is None:
            self._colorsort_obstacles()
        return self._white_obstacles

    def get_other_obstacles(self):
        # type: () -> list[Candidate]
        if self._other_obstacles is None:
            self._colorsort_obstacles()
        return self._other_obstacles

    def compute_all_obstacles(self):
        self._colorsort_obstacles()

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
