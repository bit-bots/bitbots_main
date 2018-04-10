import cv2
import numpy as np
from ball import Ball
import itertools
from .live_fcnn_03 import FCNN03


class FcnnHandler:
    def __init__(self, image, fcnn, config):
        self._image = image
        self._fcnn = fcnn
        self._rated_candidates = None
        self._sorted_rated_candidates = None
        self._top_candidate = None
        self._fcnn_output = None
        # init config
        self._debug = config['debug']
        self._threshold = config['threshold']  # minimal activation
        self._expand_stepsize = config['expand_stepsize']  #
        self._pointcloud_stepsize = config['pointcloud_stepsize']  #

        # draw the output when debug is enabled
        self.draw_debug_image()

    def get_candidates(self):
        """
        candidates are a list of tuples ((candidate),rating)
        :return:
        """
        if self._rated_candidates is None:
            self._rated_candidates = list()
            for candidate in self._get_raw_candidates():
                out = self.get_fcnn_output()
                line = out[candidate.get_center_y()]
                rating = line[candidate.get_center_x()] / 255.0
                self._rated_candidates.append((candidate, rating))
        return self._rated_candidates

    def get_top_candidate(self):
        """
        Use this to return the best candidate.
        ONLY, when never use get top candidate*s*
        When you use it once, use it all the time.
        :return: the candidate with the highest rating (candidate)
        """
        if self._top_candidate is None:
            if self._sorted_rated_candidates is None:
                if self.get_top_candidates():
                    self._top_candidate = list([max(
                        self.get_top_candidates(),
                        key=lambda x: x[1]
                    )[0]])
                else:
                    self._top_candidate = list()
            else:
                self._top_candidate = list([self._sorted_rated_candidates[0]])
        if self._top_candidate:
            return self._top_candidate[0]
        return None

    def get_top_candidates(self, count=1):
        """
        Returns the count best candidates. When you use this, using
        get_top_candidate is wrong. use it always, when you use it.
        :param count: Number of top-candidates to return
        :return: the count top candidates
        """
        if count < 1:
            raise ValueError('the count must be equal or greater 1!')
        if self._sorted_rated_candidates is None:
            self._sorted_rated_candidates = sorted(self.get_candidates(), key=lambda x: x[1])
        return self._sorted_rated_candidates[0:count-1]

    def get_fcnn_output(self):
        if self._fcnn_output is None:
            in_img = cv2.resize(self._image, (self._fcnn.input_shape[1], self._fcnn.input_shape[0]))
            out = self._fcnn.predict(list([in_img]))
            out = out.reshape(self._fcnn.output_shape[0], self._fcnn.output_shape[1])
            out = (out * 255).astype(np.uint8)
            self._fcnn_output = cv2.resize(out, (self._image.shape[1], self._image.shape[0]))
            print(self._fcnn_output.shape)
        return self._fcnn_output

    def _get_raw_candidates(self):
        """
        returns a list of candidates [(Ball), ...]
        :return: a list of candidates [(Ball), ...]
        """
        out = self.get_fcnn_output()
        r, out_bin = cv2.threshold(out, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        candidates = list()
        # creating points
        # x shape
        xshape = self._fcnn.output_shape[1]
        xlist = []
        x = 0
        while x < xshape:
            xlist.append(x)
            x += self._pointcloud_stepsize
        # y shape
        yshape = self._fcnn.output_shape[0]
        ylist = []
        y = 0
        while y < yshape:
            ylist.append(y)
            y += self._pointcloud_stepsize
        # generate carthesian product of list
        points = list(itertools.product(xlist, ylist))
        # expand points
        while points:
            point = points.pop()
            lx, uy = point
            rx, ly = point
            # expand to the left
            next_lx = max(lx - self._expand_stepsize, 0)
            while next_lx > 0 and out_bin[point[1]][next_lx]:
                lx = next_lx
                next_lx = max(lx - self._expand_stepsize, 0)
            # expand to the right
            next_rx = min(rx + self._expand_stepsize, out_bin.shape[1] - 1)
            while next_rx < out_bin.shape[1] - 1 and out_bin[point[1]][next_rx]:
                rx = next_rx
                next_rx = min(rx + self._expand_stepsize, out_bin.shape[1] - 1)
            # expand upwards
            next_uy = max(uy - self._expand_stepsize, 0)
            while next_uy > 0 and out_bin[next_uy][point[0]]:
                uy = next_uy
                next_uy = max(uy - self._expand_stepsize, 0)
            # expand downwards (the lowest y is the highest number for y)
            next_ly = min(ly + self._expand_stepsize, out_bin.shape[0] - 1)
            while next_ly < out_bin.shape[0] - 1 and out_bin[next_ly][point[0]]:
                ly = next_ly
                next_ly = min(ly + self._expand_stepsize, out_bin.shape[0] - 1)

            width, height = rx - lx, ly - uy
            candidates.append(Ball(lx, uy, width, height))
            for other_point in points:
                if lx <= other_point[0] <= rx and uy <= other_point[1] <= ly:
                    points.remove(other_point)
        return candidates

    def draw_debug_image(self):
        if not self._debug:
            return
        cv2.imshow('FCNN Output', self.get_fcnn_output())
        cv2.waitKey(1)
