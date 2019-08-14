import abc


class Candidate:
    def __init__(self, x1=0, y1=0, width=0, height=0, rating=1):
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

    def get_footpoint(self):
        # type: () -> (int, int)
        """
        returns...

        :return tuple: returns the lowest point of the candidate
        """
        return (self.get_center_x(), self.get_lower_right_y())

    def get_rating(self):
        # type: () -> float
        """
        returns rating of the candidate

        :return float: rating
        """
        return self.rating

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

    @staticmethod
    def sort_candidates(candidatelist):
        """
        Returns a sorted list of the candidates.
        The first list element is the highest rated candidate.
        :param candidatelist:
        :return: sorted candidate list
        """
        return sorted(candidatelist, key = lambda candidate: candidate.get_rating(), reverse=True)


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

    @abc.abstractmethod
    def compute(self):
        """
        Runs the most intense calculation without returning any output.
        """
        raise NotImplementedError

    def get_top_candidate(self):
        """
        Returns the best candidate.
        :param count: Number of top-candidates to return
        :return: the count top candidates
        """
        return self.get_top_candidates()[0] if self.get_top_candidates() else None


class BallDetector(CandidateFinder):
    def get_top_ball_under_convex_field_boundary(self, field_boundary_detector, y_offset=0):
        """
        Returns the best candidate under the convex field boundary.
        :return: top candidate or None if no candidate exists
        """
        # Get all balls
        balls = self.get_sorted_top_balls_under_convex_field_boundary(field_boundary_detector, y_offset)
        # Check if there are any
        if balls:
            # Return the best
            return balls[0]

    def get_sorted_top_balls_under_convex_field_boundary(self, field_boundary_detector, y_offset=0):
        """
        Returns the best candidates under the convex field boundary.
        :return: list of top candidates sorted by rating
        """

        # Get candidates
        ball_candidates = self.get_candidates()
        # Check if there are any ball candidates
        if ball_candidates:
            # Only take candidates under the convex field boundary
            balls_under_field_boundary = field_boundary_detector.candidates_under_convex_field_boundary(ball_candidates, y_offset)
            # Check if there are still candidates left
            if balls_under_field_boundary:
                # Sort candidates and take the one which has the biggest confidence
                sorted_rated_candidates = sorted(balls_under_field_boundary, key=lambda x: x.get_rating())
                return sorted_rated_candidates
        return list()
