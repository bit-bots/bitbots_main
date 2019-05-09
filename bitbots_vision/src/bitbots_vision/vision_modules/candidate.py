import abc


class Candidate:
    def __init__(self, x1=0, y1=0, width=0, height=0, rating=None):
        self._x1 = x1
        self._y1 = y1
        self._width = width
        self._height = height
        self.rating = rating

    def get_width(self):
        # type: () -> int
        """
        returns...

        :return int: width
        """
        return self._width

    def get_height(self):
        # type: () -> int
        """
        returns...

        :return int: height
        """
        return self._height

    def get_center_x(self):
        # type: () -> int
        """
        returns...

        :return int: center x
        """
        return self._x1 + int(self._width // 2)

    def get_center_y(self):
        # type: () -> int
        """
        returns...

        :return int: center y
        """
        return self._y1 + int(self._height // 2)

    def get_center_point(self):
        # type: () -> tuple[int, int]
        """
        returns...

        :return tuple[int,int]: center point
        """
        return self.get_center_x(), self.get_center_y()

    def get_diameter(self):
        # type: () -> int
        """
        returns...

        :return int: diameter
        """
        return int((self._height + self._width) // 2)

    def get_radius(self):
        # type: () -> int
        """
        returns...

        :return int: radius
        """
        return int(self.get_diameter() // 2)

    def get_upper_left_point(self):
        # type: () -> tuple[int, int]
        """
        returns...

        :return tuple[int,int]: upper left point
        """
        return self._x1, self._y1

    def get_upper_left_x(self):
        # type: () -> int
        """
        returns...

        :return int: upper left x
        """
        return self._x1

    def get_upper_left_y(self):
        # type: () -> int
        """
        returns...

        :return int: upper left y
        """
        return self._y1

    def get_lower_right_point(self):
        # type: () -> tuple[int, int]
        """
        returns...

        :return tuple[int,int]: lower right point
        """
        return self._x1 + self._width, self._y1 + self._height

    def get_lower_right_x(self):
        # type: () -> int
        """
        returns...

        :return int: lower right x
        """
        return self._x1 + self._width

    def get_lower_right_y(self):
        # type: () -> int
        """
        returns...

        :return int: lower right y
        """
        return self._y1 + self._height

    # TODO: get_rating

    def point_in_candidate(self, point):
        # type: (tuple) -> bool
        """
        Returns whether the point is in the candidate or not.
        In the process, the candidate gets treated as a rectangle.
        :param point: an x- y-int-tuple defining thhe point to inspect
        :return: whether the point is in the candidate or not
        """
        return (
                self.get_upper_left_x()
                <= point[0]
                <= self.get_upper_left_x() + self.get_width()) \
            and (
                self.get_upper_left_y()
                <= point[1]
                <= self.get_upper_left_y() + self.get_height())

    def __str__(self):
        return 'x1,y1: {0},{1} | width,height: {2},{3} | rating: {4}'.format(
            self.get_upper_left_x(),
            self.get_upper_left_y(),
            self.get_width(),
            self.get_height(),
            self.rating)


class CandidateFinder(object):

    @abc.abstractmethod
    def get_top_candidates(self, count=1):
        """
        Returns the count best candidates.
        :param count: Number of top-candidates to return
        :return: the count top candidates
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_candidates(self):
        """
        Returns a list of all candidates. Their type is Candidate.
        :return: the count top candidates
        """
        raise NotImplementedError

    def get_top_candidate(self):
        """
        Returns the best candidate.
        :param count: Number of top-candidates to return
        :return: the count top candidates
        """
        return self.get_top_candidates()[0] if self.get_top_candidates() else None
