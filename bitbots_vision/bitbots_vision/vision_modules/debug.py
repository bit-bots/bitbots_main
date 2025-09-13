from typing import Optional, Sequence

import cv2
import numpy as np
from jaxtyping import UInt8

from bitbots_vision.vision_modules.candidate import Candidate


class DebugImage:
    """
    :class:`.DebugImage` draws the images with information of the vision pipeline for debug purposes.

    It is capable of displaying the detected and convex field boundary (red and yellow lines respectively),
    the best and discarded ball candidates (green and red circles respectively),
    the goalposts (white bounding boxes) and
    different obstacles (black: unknown, red: red robot, blue: blue robot).
    """

    def __init__(self, active: bool = True):
        """
        Initialization of :class:`.DebugImage`.
        """
        self._debug_image: Optional[UInt8[np.ndarray, "h w 3"]] = None
        self.active = active

    def set_image(self, image: UInt8[np.ndarray, "h w 3"]) -> None:
        """
        Sets a new image on which the debug image is mapped.

        :param image: image the vision is currently processing
        """
        self._debug_image = image.copy()

    def draw_field_boundary(
        self, field_boundary_points: Sequence[int], color: tuple[int, int, int], thickness: int = 1
    ):
        """
        Draws a line a line that represents the given field_boundary.

        :param field_boundary_points: list of coordinates of the field_boundary
        :param color: color of the line
        :param thickness: thickness of the line
        """
        if not self.active:
            return
        assert self._debug_image is not None, "No image set"
        for i in range(len(field_boundary_points) - 1):
            cv2.line(self._debug_image, field_boundary_points[i], field_boundary_points[i + 1], color, thickness=1)  # type: ignore[call-overload]

    def draw_ball_candidates(
        self, ball_candidates: Sequence[Candidate], color: tuple[int, int, int], thickness: int = 1
    ):
        """
        Draws a circle around every coordinate where a ball candidate was found.

        :param ball_candidates: list of ball candidates with the type Candidate
        :param color: color of the circle to draw
        :param thickness: thickness of the outline
        """
        if not self.active:
            return
        assert self._debug_image is not None, "No image set"
        for candidate in ball_candidates:
            if candidate:
                cv2.circle(
                    self._debug_image,
                    (candidate.get_center_x(), candidate.get_center_y()),
                    candidate.get_radius(),
                    color,
                    thickness=thickness,
                )

    def draw_robot_candidates(
        self, robot_candidates: Sequence[Candidate], color: tuple[int, int, int], thickness: int = 1
    ):
        """
        Draws a bounding box for every given robot.

        :param robot_candidates: list of list of robot candidates with the type Candidate
        :param color: color of the outline
        :param thickness: thickness of the outline
        """
        if not self.active:
            return
        assert self._debug_image is not None, "No image set"
        for candidate in robot_candidates:
            cv2.rectangle(
                self._debug_image,
                candidate.get_upper_left_point(),
                candidate.get_lower_right_point(),
                color,
                thickness=thickness,
            )

    def draw_points(self, points: tuple[int, int], color: tuple[int, int, int], thickness: int = -1, rad: int = 2):
        """
        Draws a (line)point for every given point.

        :param points: list points
        :param color: color of the point
        :param thickness: thickness of the outline
        :param rad: radius of the point
        """
        if not self.active:
            return
        assert self._debug_image is not None, "No image set"
        for point in points:
            cv2.circle(self._debug_image, point, rad, color, thickness=thickness)  # type: ignore[call-overload]

    def draw_line_segments(
        self, segments: Sequence[tuple[int, int, int, int]], color: tuple[int, int, int], thickness: int = 2
    ):
        """
        Draws a line segment.

        :param segments: list line segments in the form (x1,y1,x2,y2)
        :param color: color of the line
        :param thickness: thickness of the line
        """
        if not self.active:
            return
        assert self._debug_image is not None, "No image set"
        for segment in segments:
            cv2.line(self._debug_image, (segment[0], segment[1]), (segment[2], segment[3]), color, thickness=2)

    def draw_mask(self, mask, color, opacity=0.5):
        if not self.active:
            return
        assert self._debug_image is not None, "No image set"
        # Make a colored image
        colored_image = np.zeros_like(self._debug_image)
        colored_image[:, :] = tuple(np.multiply(color, opacity).astype(np.uint8))

        # Compose debug image with lines
        self._debug_image = cv2.add(
            cv2.bitwise_and(self._debug_image, self._debug_image, mask=255 - mask),
            cv2.add(colored_image * opacity, self._debug_image * (1 - opacity), mask=mask).astype(np.uint8),
        )

    def get_image(self):
        """
        Get the image with the debug drawing in it.

        :return: image with debug stuff
        """
        return self._debug_image
