from .field_boundary import FieldBoundaryDetector
from .color import ColorDetector
import math
import numpy as np
import cv2


class LineDetector:
    """
    Detecting field lines in the image.
    """
    def __init__(self, config, white_detector, field_color_detector, field_boundary_detector):
        # type: (dict, ColorDetector, ColorDetector, FieldBoundaryDetector) -> None
        self._image = None
        self._white_mask = None
        self._linepoints = None
        self._linesegments = None
        self._white_detector = white_detector
        self._field_color_detector = field_color_detector
        self._field_boundary_detector = field_boundary_detector
        # Init config
        self._field_boundary_offset = config['line_detector_field_boundary_offset']
        self._linepoints_range = config['line_detector_linepoints_range']

    def set_image(self, image):
        # type: (np.matrix) -> None
        """
        refreshes the variables after receiving an image
        :param image: the current frame of the video feed
        """
        self._image = image
        self._white_mask = None
        self._linepoints = None
        # self._nonlinepoints = None
        self._linesegments = None

    def set_candidates(self, candidates):
        """
        Used for the unused hough line implementation.
        """
        # type: (list) -> None
        self._candidates = candidates

    def compute(self):
        """
        Computes the linepoints if necessary
        """
        self.get_linepoints()

    def get_linepoints(self):
        """
        Computes if necessary and returns the (cached) linepoints
        """
        # Check if points are allready cached
        if self._linepoints is None:
            # Empty line point list
            self._linepoints = list()
            # Mask white parts of the image using a white color detector
            white_masked_image = self._get_white_mask()
            # Get image shape
            imgshape = self._get_white_mask().shape

            # Get the maximum height of the field boundary
            max_field_boundary_heigth = self._field_boundary_detector.get_upper_bound(
                self._field_boundary_offset)

            # Check if there is some space between the field boundary and the image border.
            # If the field boundary equals the image border there is no need to search for line points. Also it crashes if these two are equal arrrgh...
            if max_field_boundary_heigth < imgshape[0]:
                # Get X samples
                x_list = np.random.randint(0, imgshape[1],
                                        size=self._linepoints_range, dtype=int)
                # get Y samples
                y_list = np.random.randint(max_field_boundary_heigth, imgshape[0],
                                        size=self._linepoints_range, dtype=int)
                # Check for each sample pair if their pixel in the binary white mask is true.
                for p in zip(x_list, y_list):
                    if white_masked_image[p[1]][p[0]]:
                        # Append these points to our list
                        self._linepoints.append(p)

        # Return line points
        return self._linepoints

    def get_linesegments(self):
        """
        Computes if necessary and returns the (cached) line segments (Currently unused)
        """
        # Mask white parts of the image
        img = self._get_white_mask()
        # Use hough lines algorithm to find lines in this mask
        lines = cv2.HoughLinesP(img,
                                1,
                                math.pi / 180,
                                80,
                                30,
                                minLineLength=10)
        self._linesegments = []
        if lines is None:
            return self._linesegments
        # Iterate over hough lines
        for l in lines:
            # Iterate over start and end
            for x1, y1, x2, y2 in l:
                # Check if start or end is in any of the candidates
                in_candidate = False
                for candidate in self._candidates:
                    if candidate and (
                            candidate.point_in_candidate((x1, x2)) or
                            candidate.point_in_candidate((x2, y2))):
                        in_candidate = True
                        break
                # Check if start and end is under field_boundary
                under_field_boundary = self._field_boundary_detector.point_under_field_boundary(
                    (x1, y1), self._field_boundary_offset) and \
                                self._field_boundary_detector.point_under_field_boundary(
                                    (x1, y1), self._field_boundary_offset)
                # Add segment if it is not in any candidate and it starts and ends under the field boundary
                if not in_candidate and under_field_boundary:
                    self._linesegments.append((x1, y1, x2, y2))
        return self._linesegments

    def _get_white_mask(self):
        """
        Generates a white mask that not contains pixels in the green field or above the field boundary
        """
        # Check if it is cached
        if self._white_mask is None:
            # Only take parts that are under not green and the field boundary
            # Get green mask
            green_mask = self._field_color_detector.get_mask_image()
            # Noise reduction on the green field mask
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel=np.ones((3, 3)), iterations=1)
            # Invert and scale the field mask
            not_green_mask = np.ones_like(green_mask) - (np.floor_divide(green_mask, 255))
            # Get part under the field boundary as white mask
            field_boundary_mask = self._field_boundary_detector.get_mask()
            # Get not green points under field boundary
            possible_line_locations = cv2.bitwise_and(not_green_mask, not_green_mask, mask=field_boundary_mask)
            # Get white points that are not above the field boundary or in the green field
            self._white_mask = self._white_detector.mask_bitwise(possible_line_locations)
        return self._white_mask

    @staticmethod
    def filter_points_with_candidates(linepoints, candidates):
        """
        Filters line points with candidates.

        :param linepoints: Line Points
        :param candidates: Detected candidates
        :return: Filtered line points
        """
        filtered_linepoints = []
        for linepoint in linepoints:
            # TODO: use list comprehension instead of map
            linepoint_not_in_candidate = [not candidate.point_in_candidate(linepoint) for candidate in candidates]
            if all(linepoint_not_in_candidate):
                filtered_linepoints.append(linepoint)
        return filtered_linepoints
