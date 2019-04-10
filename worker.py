#!/usr/bin/env python2
import numpy as np
import cv2
from silx.image import sift
import math
from functools import partial


class VisualCompass:
    """
    Interface for an Visual compass, that gets new images and calculates an horizontal angle with a corresponding
    confidence.
    """
    def process_image(self, image, resultCB=None, debugCB=None):
        # type: (np.array, func, func) -> None
        """
        Processes an image and updates the internal state

        :param image:
        :param resultCB:
        :param debugCB:
        :return:
        """
        pass

    def set_config(self, config):
        # type: (dict) -> None
        """

        :param config:
        :return:
        """
        pass

    def set_truth(self, angle, image):
        # type: (float, np.array) -> None
        """

        :param angle:
        :param image:
        :return:
        """
        pass

    def get_side(self):
        # type: () -> (float, float)
        """

        :return:
        """
        pass

class BinaryCompass(VisualCompass):
    """
    This compass only compares two sides, hence two base images.
    """

    def __init__(self, config):
        self.set_config(config)
        self.groundTruth = [None, None]
        self.siftPlan = None
        self.state = (None, None)
        self.debugger = Debug()

    def _init_sift(self, shape, dtype):
        self.siftPlan = sift.SiftPlan(shape, dtype, devicetype="CPU")
        self.matchPlan = sift.MatchPlan()

    def process_image(self, image, resultCB=None, debugCB=None):
        if None in self.groundTruth:
            return

        keypoints = self._get_keypoints(image)
        matches = self._compare(keypoints)
        self.state = self._compute_state(matches)
        if resultCB is not None:
            resultCB(*self.state)

        if debugCB is not None:
            self.debugger.print_debug_info(image, matches, keypoints, self.state, debugCB)

    def set_config(self, config):
        pass

    def set_truth(self, angle, image):
        if angle == 0:
            self.groundTruth[0] = self._get_keypoints(image)
        elif angle == math.pi:
            self.groundTruth[1] = self._get_keypoints(image)

    def get_side(self):
        return self.state

    def _compare(self, keypoints):
        # only process if keypoints are found in image
        if not keypoints.shape[0]:
            return 0, 0

        matches = map(partial(self.matchPlan, keypoints).func, self.groundTruth)
        return matches

    def _get_keypoints(self, image):
        if self.siftPlan is None:
            self._init_sift(image.shape, image.dtype)

        return self.siftPlan.keypoints(image)

    def _compute_state(self, matches):
        match_counts = [x.shape[0] for x in matches]
        angle = 0 if match_counts[0] > match_counts[1] else math.pi

        confidence = abs(match_counts[0] - match_counts[1])/(float(sum(match_counts) + 1))
        return angle, confidence


class Debug:

    def __init__(self):
        pass

    def print_debug_info(self, image, matches, keypoints, state, callback):
        image_with_matches = self.plot(image, self.convert_match(matches[0][:, 0]), (255, 0, 0), 6)
        image_with_matches = self.plot(image_with_matches, self.convert_match(matches[1][:, 0]), (0, 255, 0), 4)

        font = cv2.FONT_HERSHEY_SIMPLEX
        bottom_left_corner_of_text = (10, 35)
        font_scale = 1
        font_color = (255, 255, 255)
        line_type = 2

        cv2.putText(image_with_matches, "SIDE {}".format(state),
                    bottom_left_corner_of_text,
                    font,
                    font_scale,
                    font_color,
                    line_type)

        callback(self.plot(image_with_matches, keypoints, (0, 0, 255), 2))

    def convert_match(self, kps):
        d = np.dtype((np.record, [('x', '<f4'), ('y', '<f4'), ('scale', '<f4'), ('angle', '<f4'), ('desc', 'u1', (128,))]))
        return kps.astype(d)

    def plot(self, image, kp, color, size):
        for i in range(kp.shape[0]):
            cv2.circle(image, (kp[i].x, kp[i].y), size + int(kp[i].scale), color, thickness=2)
        return image