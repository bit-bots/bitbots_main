
import sys
from random import randint
from .field_boundary import FieldBoundaryDetector
from .candidate import Candidate
from .color import ColorDetector
import math
import numpy as np
import cv2


class LineDetector:
    def __init__(self, white_detector, field_color_detector, field_boundary_detector, config):
        # type: (ColorDetector, ColorDetector, FieldBoundaryDetector, dict) -> None
        self._image = None
        self._preprocessed_image = None
        self._linepoints = None
        # self._nonlinepoints = None  # these are points that are not found on a line, helpful for localisation
        self._linesegments = None
        self._white_detector = white_detector
        self._field_color_detector = field_color_detector
        self._field_boundary_detector = field_boundary_detector
        # init config
        self._field_boundary_offset = config['line_detector_field_boundary_offset']
        self._linepoints_range = config['line_detector_linepoints_range']
        self._blur_kernel_size = config['line_detector_blur_kernel_size']

    def set_image(self, image):
        self._image = image
        self._preprocessed_image = None
        self._linepoints = None
        # self._nonlinepoints = None
        self._linesegments = None

    def set_candidates(self, candidates):
        # type: (list) -> None
        self._candidates = candidates

    def compute_linepoints(self):
        # if self._linepoints is None or self._nonlinepoints is None:
        if self._linepoints is None:
            self._linepoints = list()
            imgshape = self._get_preprocessed_image().shape
            white_masked_image = self._white_detector.mask_image(
                self._get_preprocessed_image())

            # get the maximum height of the field boundary
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

    def get_linepoints(self):
        self.compute_linepoints()
        return self._linepoints

    def get_linesegments(self):

        img = self._white_detector.mask_image(self._get_preprocessed_image())
        lines = cv2.HoughLinesP(img,
                                1,
                                math.pi / 180,
                                80,
                                30,
                                minLineLength=10)
        self._linesegments = []
        if lines is None:
            return self._linesegments
        for l in lines:
            for x1, y1, x2, y2 in l:
                # check if start or end is in any of the candidates
                in_candidate = False
                for candidate in self._candidates:
                    if candidate and (
                            candidate.point_in_candidate((x1, x2)) or
                            candidate.point_in_candidate((x2, y2))):
                        in_candidate = True
                        break

                # check if start and end is under field_boundary
                under_field_boundary = self._field_boundary_detector.point_under_field_boundary(
                    (x1, y1), self._field_boundary_offset) and \
                                self._field_boundary_detector.point_under_field_boundary(
                                    (x1, y1), self._field_boundary_offset)

                if not in_candidate and under_field_boundary:
                    self._linesegments.append((x1, y1, x2, y2))

        return self._linesegments

    def _get_preprocessed_image(self):
        if self._preprocessed_image is None:
            self._preprocessed_image = self._image.copy()
            # fill everything above field_boundary black
            hpoints = np.array([[(0, 0)] +
                                self._field_boundary_detector.get_field_boundary_points(self._field_boundary_offset) +
                                [(self._preprocessed_image.shape[1] - 1, 0)]])
            cv2.fillPoly(self._preprocessed_image, hpoints, 000000)

            # get negated green mask
            green_mask = self._field_color_detector.mask_image(self._image)
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel=np.ones((3, 3)), iterations=1)

            green_mask = np.ones_like(green_mask) - (np.floor_divide(green_mask, 255))
            self._preprocessed_image = cv2.bitwise_and(self._preprocessed_image, self._preprocessed_image, mask=green_mask)

        return self._preprocessed_image

    @staticmethod
    def filter_points_with_candidates(linepoints, candidates):
        filtered_linepoints = linepoints
        for candidate in candidates:
            filtered_linepoints = [linepoint for linepoint in linepoints if not candidate.point_in_candidate(linepoint)]
        return filtered_linepoints
