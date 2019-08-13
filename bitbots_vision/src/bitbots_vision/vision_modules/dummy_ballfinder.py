from .candidate import CandidateFinder, BallDetector


class DummyBallDetector(BallDetector):
    """
    Dummy ball detector that we use if we want the run vision to without neural network e.g..
    """
    def __init__(self):
        self._detected_candidates = []
        self._sorted_candidates = []
        self._top_candidate = None

    def set_image(self, image):
        """
        Method to satisfy the interface.
        Actually does nothing.
        :param image: current vision image
        """
        pass

    def compute(self):
        """
        Method to satisfy the interface.
        Actually does nothing, except the extrem complicated command 'pass'.
        """
        pass

    def get_candidates(self):
        """
        Method to satisfy the interface.
        Actually does something. It returns an empty list.
        :return: a empty list
        """
        return self._detected_candidates

    def get_top_candidates(self, count=1):
        """
        Method to satisfy the interface.
        It returns an empty list.
        :param count: how many of zero top candidates do you want?
        :return: a empty list
        """
        return self._sorted_candidates
