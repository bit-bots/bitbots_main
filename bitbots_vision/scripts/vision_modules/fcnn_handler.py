import numpy as np
import cv2


class FcnnHandler:
    def __init__(self, image, fcnn):
        self._image = image
        self._fcnn = fcnn
        self._rated_candidates = None
        self._sorted_rated_candidates = None
        self._top_candidate = None
        self._fcnn_output = None

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
        if not self._top_candidate:
            if not self._sorted_rated_candidates:
                self._top_candidate = max(
                    self.get_top_candidates(),
                    key=lambda x: x[1]
                )[0]
            else:
                self._top_candidate = self._sorted_rated_candidates[0]
        return self._top_candidate

    def get_fcnn_output(self):
        if not self._fcnn_output:
            pass  # TODO: implement this stuff
        return self._fcnn_output

