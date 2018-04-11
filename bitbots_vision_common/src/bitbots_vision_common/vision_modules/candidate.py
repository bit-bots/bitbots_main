import abc


class Candidate:
    def __init__(self, x1=0, y1=0, width=0, height=0, rating=None):
        self._x1 = x1
        self._y1 = y1
        self._width = width
        self._height = height
        self.rating = rating

    def get_width(self):
        return self._width

    def get_height(self):
        return self._height

    def get_center_x(self):
        return self._x1 + int(self._width // 2)

    def get_center_y(self):
        return self._y1 + int(self._height // 2)

    def get_center_point(self):
        return self.get_center_x(), self.get_center_y()

    def get_diameter(self):
        return int((self._height + self._width) // 2)

    def get_radius(self):
        return int(self.get_diameter() // 2)

    def get_upper_left_point(self):
        return self._x1, self._y1

    def get_upper_left_x(self):
        return self._x1

    def get_upper_left_y(self):
        return self._y1


class CandidateFinder(object):

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
        return self.get_top_candidates()
