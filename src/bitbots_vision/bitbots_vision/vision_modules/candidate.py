from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    pass


from rclpy import logging

logger = logging.get_logger("vision_candidate")


class Candidate:
    """
    A :class:`.Candidate` is a representation of an arbitrary object in an image.
    It is very similar to bounding boxes but with an additional rating.

    This class provides several getters for different properties of the candidate.
    """

    def __init__(self, x1=0, y1=0, width=0, height=0, rating=None):
        """
        Initialization of :class:`.Candidate`.

        :param int x1: Horizontal part of the coordinate of the top left corner of the candidate
        :param int y1: Vertical part of the coordinate of the top left corner of the candidate
        :param int width: Horizontal size
        :param int height: Vertical size
        :param float rating: Confidence of the candidate
        """
        self._x1 = x1
        self._y1 = y1
        self._width = width
        self._height = height
        self._rating = rating

    def get_width(self):
        # type: () -> int
        """
        :return int: Width of the candidate bounding box.
        """
        return self._width

    def get_height(self):
        # type: () -> int
        """
        :return int: Height of the candidate bounding box.
        """
        return self._height

    def get_center_x(self):
        # type: () -> int
        """
        :return int: Center x coordinate of the candidate bounding box.
        """
        return self._x1 + int(self._width // 2)

    def get_center_y(self):
        # type: () -> int
        """
        :return int: Center y coordinate of the candidate bounding box.
        """
        return self._y1 + int(self._height // 2)

    def get_center_point(self):
        # type: () -> tuple[int, int]
        """
        :return tuple[int,int]: Center point of the bounding box.
        """
        return self.get_center_x(), self.get_center_y()

    def get_diameter(self):
        # type: () -> int
        """
        :return int: Mean diameter of the candidate.
        """
        return int((self._height + self._width) // 2)

    def get_radius(self):
        # type: () -> int
        """
        :return int: Mean radius of the candidate.
        """
        return int(self.get_diameter() // 2)

    def get_upper_left_point(self):
        # type: () -> tuple[int, int]
        """
        :return tuple[int,int]: Upper left point of the candidate.
        """
        return self._x1, self._y1

    def get_upper_left_x(self):
        # type: () -> int
        """
        :return int: Upper left x coordinate of the candidate.
        """
        return self._x1

    def get_upper_left_y(self):
        # type: () -> int
        """
        :return int: Upper left y coordinate of the candidate.
        """
        return self._y1

    def get_lower_right_point(self):
        # type: () -> tuple[int, int]
        """
        :return tuple[int,int]: Lower right point of the candidate.
        """
        return self._x1 + self._width, self._y1 + self._height

    def get_lower_right_x(self):
        # type: () -> int
        """
        :return int: Lower right x coordinate of the candidate.
        """
        return self._x1 + self._width

    def get_lower_right_y(self):
        # type: () -> int
        """
        :return int: Lower right y coordinate of the candidate.
        """
        return self._y1 + self._height

    def get_lower_center_point(self) -> tuple[int, int]:
        """
        :return tuple: Returns the lowest point of the candidate. The point is horizontally centered inside the candidate.
        """
        return (self.get_center_x(), self.get_lower_right_y())

    def set_rating(self, rating):
        # type: (float) -> None
        """
        :param float rating: Rating to set.
        """
        if self._rating is not None:
            logger.warning("Candidate rating has already been set.")
            return
        self._rating = rating

    def get_rating(self):
        # type: () -> float
        """

        :return float: Rating of the candidate
        """
        return self._rating

    def point_in_candidate(self, point):
        # type: (tuple) -> bool
        """
        Returns whether the point is in the candidate or not.
        In the process, the candidate gets treated as a rectangle.

        :param point: An x-y-int-tuple defining the point to inspect.
        :return bool: Whether the point is in the candidate or not.
        """
        return (self.get_upper_left_x() <= point[0] <= self.get_upper_left_x() + self.get_width()) and (
            self.get_upper_left_y() <= point[1] <= self.get_upper_left_y() + self.get_height()
        )

    def set_in_mask(self, mask, value=0, grow=1):
        """
        Sets the bounding box of this candidate in the given mask to the given value.

        :param mask: Binary mask with the shape of the input image
        :param value: The value of the bounding box region
        :param grow: A scalar which defines how much arround the box is also removed
        :returns mask: The input mask without this candidate
        """
        width = int(self.get_width() * grow * 0.5)
        height = int(self.get_height() * grow * 0.5)
        mask[
            max(self.get_center_y() - height, 0) : min(self.get_center_y() + height, mask.shape[0]),
            max(self.get_center_x() - width, 0) : min(self.get_center_x() + width, mask.shape[1]),
        ] = value
        return mask

    @staticmethod
    def sort_candidates(candidatelist):
        """
        Returns a sorted list of the candidates.
        The first list element is the highest rated candidate.

        :param [Candidate] candidatelist: List of candidates
        :return: List of candidates sorted by rating, in descending order
        """
        return sorted(candidatelist, key=lambda candidate: candidate.get_rating(), reverse=True)

    @staticmethod
    def select_top_candidate(candidatelist):
        """
        Returns the highest rated candidate.

        :param candidatelist: List of candidates
        :return Candidate: Top candidate
        """
        if candidatelist:
            return Candidate.sort_candidates(candidatelist)[0]
        else:
            return None

    @staticmethod
    def rating_threshold(candidatelist, threshold):
        """
        Returns list of all candidates with rating above given threshold.

        :param [Candidate] candidatelist: List of candidates to filter
        :param float threshold: Filter threshold
        :return [Candidate]: Filtered list of candidates
        """
        return [candidate for candidate in candidatelist if candidate.get_rating() > threshold]

    def __str__(self):
        """
        Returns string representation of candidate.

        :return str: String representation of candidate
        """
        return f"x1,y1: {self.get_upper_left_x()},{self.get_upper_left_y()} | width,height: {self.get_width()},{self.get_height()} | rating: {self._rating}"

    @classmethod
    def from_x1y1x2y2(cls, x1, y1, x2, y2, rating=None):
        width = abs(x1 - x2)
        height = abs(y1 - y2)
        return cls(min(x1, x2), min(y1, y2), width, height, rating)
