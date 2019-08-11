from .candidate import Candidate, CandidateFinder
from .obstacle import ObstacleDetector
from .color import ColorDetector

from VisionExtensions import expandPoint


class ObstaclePostDetector(CandidateFinder):
    """
    searches goalposts based on the obstacle detector.
    """

    def __init__(self, obstacle_detector, white_color_detector, config):
        # type: (ObstacleDetector, ColorDetector, dict) -> None
        self._obstacle_detector = obstacle_detector
        self._white_color_detector = white_color_detector
        self._expand_stepsize = config['expand_stepsize']
        self._white_threshold = config['white_threshold']

        self._white_masked_image = None
        self._image = None
        self._goalposts = None

    def set_image(self, image):
        self._image = image
        self._goalposts = None
        self._white_masked_image = self._white_color_detector.mask_image(image)

    def compute(self):
        """
        Method to satisfy the interface
        """
        pass

    def get_candidates(self):
        # type: () -> list[Candidate]
        if self._goalposts is None:
            self._goalposts = self._filter_posts(
                self._obstacle_detector.get_white_obstacles())
        return self._goalposts

    def get_top_candidates(self, count=1):
        return self.get_candidates()

    def _filter_posts(self, goalpost_candidates):
        # type: (list[Candidate]) -> list[Candidate]
        goalposts = list()
        for post in goalpost_candidates:
            expanded_candidate = expandPoint(
                self._white_masked_image,
                post.get_center_point(),
                self._white_threshold,
                self._expand_stepsize
            )
            if expanded_candidate:
                goalposts.append(expanded_candidate)
        return goalposts

