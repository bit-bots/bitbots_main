import itertools
import numpy as np
import rclpy
from .candidate import CandidateFinder, Candidate
from .color import ColorDetector
from .field_boundary import FieldBoundaryDetector


class ObstacleDetector(CandidateFinder):
    """
    The obstacle detection module is a :class:`bitbots_vision.vision_modules.candidate.CandidateFinder` that finds obstructions, e.g. robots.
    In order to perform its task it uses the normal or convex field_boundary of a :class:`bitbots_vision.vision_modules.field_boundary.FieldBoundaryDetector` depending on the method used.
    Given that the field boundary contains dents where objects obstruct the edge of the field and consists of a list of points,
    the obstacle detection module can find these objects by comparing the height of adjacent field boundary-points.
    Alternatively objects can be found by measuring the distance between the ordinary field boundary and
    the convex field boundary which is a slightly less efficient but more accurate method.
    """
    def __init__(self, config, field_boundary_detector):
        """
        Initialization of the ObstacleDetector.

        :param config: Configuration as defined in visionparams.yaml
        :param field_boundary_detector: locates the field_boundary
        """
        # type: (dict, ColorDetector, ColorDetector, ColorDetector, FieldBoundaryDetector) -> None
        # Set used detectors
        self._field_boundary_detector = field_boundary_detector

        # Set own config parameters
        self.set_config(config)

        # Set values to None needed for caching
        self._image = None

        # Set output to None
        self._obstacles = None

        # Set if values should be cached
        self._caching = config['caching']

    def set_config(self, config):
        self._field_boundary_diff_threshold = config['obstacle_field_boundary_diff_threshold']
        self._candidate_field_boundary_offset = config['obstacle_candidate_field_boundary_offset']
        self._candidate_min_width = config['obstacle_candidate_min_width']
        self._candidate_max_width = config['obstacle_candidate_max_width']
        self._finder_step_length = config['obstacle_finder_step_length']
        self._obstacle_finder_method = config['obstacle_finder_method']
        self._distance_value_increase = config['obstacle_finder_value_increase']
        self.active = config['obstacle_active']

    def set_image(self, image):
        """
        Set a image for the obstacle detector. This also resets the caches.

        :param image: current vision image
        """
        # Check if image has been set
        if np.array_equal(image, self._image):
            return
        # Set image
        self._image = image
        # Reset cached values
        self._obstacles = None

    def get_top_candidates(self, count=1):
        """
        This is bullshit for the abstract class.

        :param count: number of candidates
        """
        return self.get_candidates()[0:count]

    def get_candidates(self):
        """
        Calculate and return obstacles.
        The methods are selected depending on the config.
        """
        # Exit if detector is not active
        if not self.active:
            return []
        # Check if allready cached
        if self._obstacles is None or not self._caching:
            # Select calculation method
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
        Finds candidates by comparing the height of adjacent field_boundary points
        faster, less accurate alternative to get_candidates_convex.

        :return: candidate(int: x upper left point, int: y upper left point, int: width, int: height)
        """
        if self._obstacles is None or not self._caching:
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
                                a[1] - max(0, obstacle_begin[1] - self._candidate_field_boundary_offset)
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
                        a[1] - max(0, obstacle_begin[1] - self._candidate_field_boundary_offset)
                    )
                )
        return self._obstacles

    def _obstacle_detector_convex(self):
        # type: () -> list[Candidate]
        """
        Finds candidates using the difference of the convex field_boundary and the normal field_boundary.
        Alternative to get_candidates (more accurate, about 0.0015 seconds slower).

        :return: candidate(int: x upper left point, int: y upper left point, int: width, int: height)
        """
        # TODO: fix rarely finding not existent obstacles at the edge (vectors + orthogonal distance?)
        # TODO: increase step length before beginning of obstacle has been found, decrease it afterwards
        # TODO: interpolate individual points instead of the whole list (see get_full_field_boundary)
        if self._obstacles is None or not self._caching:
            self._obstacles = list()
            obstacle_begin = None
            # the ordinary field_boundary and convex_field_boundary consist out of a limited amount of points (usually 30).
            # the full_field_boundary/full_convex_field_boundary have interpolated points
            # to have as many points as the width of the picture.
            full_convex_field_boundary = np.array(self._field_boundary_detector.get_full_convex_field_boundary()).astype(int)
            full_field_boundary = np.array(self._field_boundary_detector.get_full_field_boundary()).astype(int)
            # calculates the distance between the points of the full_field_boundary and full_convex_field_boundary
            # field_boundary_distance is a list of distances with the index being the corresponding x-coordinate
            field_boundary_distance = full_field_boundary - full_convex_field_boundary
            # threshold determines the minimum distance of the two field_boundarys for an object to be found
            # minWidth determines the minimum width of potential objects to be identified as candidates
            # step is the length of one step in pixel: lager step -> faster, but more inaccurate
            # TODO: value of minWidth and step has to be tested
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
                        w = i - x
                        if w > min_width:
                            y = max(0, obstacle_begin[1] - self._candidate_field_boundary_offset)
                            h = np.round(np.max(full_field_boundary[x:i]) - y)
                            if h < 0:
                                rospy.logerr('Negative obstacle height', logger_name="vision_obstacle_detector")
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
                        rospy.logerr('Negative obstacle height', logger_name="vision_obstacle_detector")
                    self._obstacles.append(Candidate(x, y, w, h))
        return self._obstacles

    def _obstacle_detector_distance(self):
        # type: () -> list[Candidate]
        """
        Finds candidates using the difference of the convex field_boundary and the normal field_boundary.
        Detection of obstacles depends on their height in image and therefore their distance.

        :return: candidate(int: x upper left point, int: y upper left point, int: width, int: height)
        """
        self._obstacles = list()
        obstacle_begin = None

        full_convex_field_boundary = np.array(
            self._field_boundary_detector.
            get_full_convex_field_boundary()).astype(int)
        full_field_boundary = np.array(self._field_boundary_detector.get_full_field_boundary()).astype(int)

        # TODO: value of minWidth and step has to be tested
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
                    self._build_and_save_obstacle_candidate(obstacle_begin,
                        i,
                        full_field_boundary,
                        full_convex_field_boundary,
                        start_min_width,
                        start_max_width,
                        distance_value_increase)
                    obstacle_begin = None
        if obstacle_begin:
            # obstacle began but never ended (problematic edge-case):
            # candidate(x upper left point, y upper left point, width, height)
            i = pic_width - step  # we have to reinitialize i because it was only usable in the for-loop
            self._build_and_save_obstacle_candidate(obstacle_begin,
                i,
                full_field_boundary,
                full_convex_field_boundary,
                start_min_width,
                start_max_width,
                distance_value_increase)
        return self._obstacles

    def _build_and_save_obstacle_candidate(self, obstacle_begin, i, full_field_boundary, full_convex_field_boundary, start_min_width, start_max_width, distance_value_increase):
        """
        Creates a candidate.

        :param obstacle_begin: X position of the obstacle begining
        :param i: X position of the obstacle ending
        :param full_field_boundary: Mapping a field boundary y value to every x value
        :param full_convex_field_boundary: Mapping a convex field boundary y value to every x value
        :param start_min_width: min width
        :param start_max_width: max width
        :param distance_value_increase: distance value increase
        """
        # TODO: rename i and method -> refactor (return candidate)
        x = obstacle_begin[0]
        # Calculating width of the object
        w = i - x
        y = max(0, obstacle_begin[1] - self._candidate_field_boundary_offset)  # Top
        h = np.round(np.max(full_field_boundary[x:i]) - y)
        current_min_width = start_min_width + int((full_convex_field_boundary[i] - h) * distance_value_increase)
        current_max_width = start_max_width + int((full_convex_field_boundary[i] - h) * distance_value_increase)
        # Check if obstacle is not too small and not too big
        if current_min_width < w < current_max_width:
            if h < 0:
                rospy.logerr('Negative obstacle height', logger_name="vision_obstacle_detector")
            # Append with new candidate
            self._obstacles.append(Candidate(x, y, w, h, 1))

    def compute(self):
        """
        Calculate all obstacles and sorts them by colors.
        """
        self.get_candidates()

class ColorObstacleDetector(CandidateFinder):
    """
    Wraps an obstacle detector to return only obstacles of a certain color.
    """
    def __init__(self, obstacle_detector, color_detector=None, threshold=0, subtractors=[]):
        # type: (ObstacleDetector) -> None
        """
        Initialization of the color obstacle detector.

        :param color_detector: checks whether a color is part of the color mask
        :param subtractors: list of obstacle detectors. Their detections will be excluded from this detector
        """
        # Set used detectors
        self._obstacle_detector = obstacle_detector
        self._color_detector = color_detector

        # Init obstacle list
        self._obstacles = None

        # List of subtractors
        self._subtractors = subtractors

        self._color_threshold = threshold

    def set_image(self, image):
        # type: (np.ndarray) -> None
        """
        Set the current vision image.

        :param image: image the current image vision
        """
        self._obstacle_detector.set_image(image)

        # Reset cache
        self._obstacles = None

    def get_candidates(self):
        # type: () -> list(Candidate)
        """
        :return: list with all obstacles of this color
        """
        if self._obstacles is None:
            obstacles = []
            # Check if we filter for a color
            if self._color_detector is None:
                # If not pass the obstacle detector output
                obstacles = self._obstacle_detector.get_candidates()
            else:
                # Calculate HSV masks
                self._color_mask = self._color_detector.get_mask_image()
                # Iterate over all found obstacles
                for obstacle in self._obstacle_detector.get_candidates():
                    # Calc the colorness of the candidate
                    colorness = np.mean(
                        self._color_mask[
                            obstacle.get_upper_left_y():obstacle.get_lower_right_y(),
                            obstacle.get_upper_left_x():obstacle.get_lower_right_x()
                        ]
                    )
                    # Add obstacles over the threshold to the output list
                    if colorness > self._color_threshold:
                        obstacles.append(obstacle)

            # Get the ignored obstacles. This way an obstacle can not be a red and blue obstacle, as this would be filtered.
            ignored_obstacles = itertools.chain.from_iterable(sub_det.get_candidates() for sub_det in self._subtractors)
            # Subtract them from our detections
            self._obstacles =  list(set(obstacles) - set(ignored_obstacles))

        return self._obstacles

    def compute(self):
        # type: () -> None
        """
        Starts computation of the obstacles (cached).
        """
        self._obstacle_detector.compute()
        self.get_candidates()
