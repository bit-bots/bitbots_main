import numpy as np
import cv2


class FcnnHandler:
    def __init__(self, image, fcnn, config):
        self._image = image
        self._fcnn = fcnn
        self._rated_candidates = None
        self._sorted_rated_candidates = None
        self._top_candidate = None
        self._fcnn_output = None
        # init config
        self._threshold = config['threshold']  # minimal actovation
        self._pointcount = config['pointcount']  # number of points in the image, which get checked to find candidates

    def get_candidates(self):
        """
        candidates are a list of tuples ((x, y),rating)
        :return:
        """
        if not self._rated_candidates:
            output = self.get_fcnn_output()
            pass  # TODO: implement this stuff
        return self._rated_candidates

    def get_top_candidate(self):
        """
        Use this to return the best candidate.
        ONLY, when never use get top candidate*s*
        When you use it once, use it all the time.
        :return: the candidate with the highest rating (candidate)
        """
        if not self._top_candidate:
            if not self._sorted_rated_candidates:
                self._top_candidate = max(
                    self.get_top_candidates(),
                    key=lambda x: x[1]
                )[0]
            else:
                self._top_candidate = self._sorted_rated_candidates[0]
        return self._top_candidate

    def get_top_candidates(self, count=1):
        """
        Returns the count best candidates. When you use this, using
        get_top_candidate is wrong. use it always, when you use it.
        :param count: Number of top-candidates to return
        :return: the count top candidates
        """
        if count < 1:
            raise ValueError('the count must be equal or greater 1!')
        if not self._sorted_rated_candidates:
            self._sorted_rated_candidates = self.get_top_candidates()\
                .sorted(key=lambda x: x[1])
        return self._sorted_rated_candidates[0:count-1]

    def get_fcnn_output(self):
        if not self._fcnn_output:
            pass  # TODO: implement this stuff
        return self._fcnn_output

    def get_raw_candidates(self):
        out = self.get_fcnn_output()
        threshold_out = cv2.threshold(out, self._threshold, 255, cv2.THRESH_BINARY)
        pass
