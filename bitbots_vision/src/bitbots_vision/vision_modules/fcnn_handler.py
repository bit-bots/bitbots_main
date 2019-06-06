import cv2
from cv_bridge import CvBridge
from humanoid_league_msgs.msg import ImageWithRegionOfInterest
import VisionExtensions
import numpy as np
from .candidate import CandidateFinder, Candidate
import itertools
import random
import rospy
from .live_fcnn_03 import FCNN03
from .debug import DebugPrinter


class FcnnHandler(CandidateFinder):
    """
    This handles FCNNs, meaning it finds and rates candidates in their output.

    Example configuration (put these values into the config dictionary or just
    copy them into the vision package config):

    ball_fcnn:
        model_path: '/models/fcnn03'
        debug: true  # only active when true and vision/debug is true, too!
        threshold: .5  # minimal value for a candidate to be considered
        expand_stepsize: 4
        pointcloud_stepsize: 10
        shuffle_candidate_list: true  # shuffles the list of possible candidate points
        min_candidate_diameter: 10
        max_candidate_diameter: 200
        candidate_refinement_iteration_count: 1
    """

    def __init__(self, fcnn, field_boundary_detector, config, debug_printer):
        self._image = None
        self._fcnn = fcnn
        self._field_boundary_detector = field_boundary_detector
        self._rated_candidates = None
        self._sorted_rated_candidates = None
        self._top_candidate = None
        self._fcnn_output = None
        self._debug_printer = debug_printer
        self.bridge = CvBridge()
        # init config
        self.set_config(config)


    def set_image(self, image):
        self._image = image
        self._rated_candidates = None
        self._sorted_rated_candidates = None
        self._top_candidate = None
        self._fcnn_output = None

        # draw the output when debug is enabled
        self.draw_debug_image()

    def set_config(self, config):
        self._debug = config['debug']
        self._threshold = config['threshold']  # minimal activation
        self._expand_stepsize = config['expand_stepsize']  #
        self._pointcloud_stepsize = config['pointcloud_stepsize']  #
        self._shuffle_candidate_list = config['shuffle_candidate_list']
        self._min_candidate_diameter = config['min_candidate_diameter']
        self._max_candidate_diameter = config['max_candidate_diameter']
        self._candidate_refinement_iteration_count = \
            config['candidate_refinement_iteration_count']
        self._field_boundary_offset = config['publish_field_boundary_offset']


    def get_candidates(self):
        """
        candidates are a list of tuples ((candidate),rating)
        :return:
        """
        if self._rated_candidates is None:
            self._rated_candidates = list()
            for candidate in self._get_raw_candidates_cpp():
                out = self.get_fcnn_output()
                candidate.rating = np.mean(
                    out[
                        candidate.get_upper_left_y():
                        candidate.get_upper_left_y() + candidate.get_height(),
                        candidate.get_upper_left_x():
                        candidate.get_upper_left_x() + candidate.get_width()]
                ) / 255.0
                if self.inspect_candidate(candidate):
                    self._rated_candidates.append(candidate)
        return self._rated_candidates

    def inspect_candidate(self, candidate):
        # type: (Candidate) -> bool
        return candidate.rating >= self._threshold \
               and self._min_candidate_diameter \
               <= candidate.get_diameter() \
               <= self._max_candidate_diameter

    def compute_top_candidate(self):
        if self._top_candidate is None:
            if self._sorted_rated_candidates is None:
                if self.get_top_candidates():
                    self._top_candidate = list([max(
                        self.get_top_candidates(),
                        key=lambda x: x.rating
                    )])
                else:
                    self._top_candidate = list()  # empty list -> initialized, but no candidate available
            else:
                self._top_candidate = list([self._sorted_rated_candidates[0]])

    def get_top_candidate(self):
        """
        Use this to return the best candidate.
        ONLY, when never use get top candidate*s*
        When you use it once, use it all the time.
        :return: the candidate with the highest rating (candidate)
        """
        self._debug_printer.info('get top candidate compute', 'fcnn')
        start = cv2.getTickCount()
        self.compute_top_candidate()
        end = cv2.getTickCount()
        self._debug_printer.info('->' + str((end - start) / cv2.getTickFrequency()), 'fcnn')
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
            self._sorted_rated_candidates = sorted(self.get_candidates(), key=lambda x: x.rating)
        return self._sorted_rated_candidates[0:count]

    def get_fcnn_output(self):
        if self._fcnn_output is None:
            in_img = cv2.resize(self._image, (self._fcnn.input_shape[1], self._fcnn.input_shape[0]))
            in_img = cv2.cvtColor(in_img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            out = self._fcnn.predict(list([in_img]))
            out = out.reshape(self._fcnn.output_shape[0], self._fcnn.output_shape[1])
            out = (out * 255).astype(np.uint8)
            self._fcnn_output = cv2.resize(out, (self._image.shape[1], self._image.shape[0]))
        return self._fcnn_output

    def _get_raw_candidates_cpp(self):

        start = cv2.getTickCount()
        out = self.get_fcnn_output()
        end = cv2.getTickCount()
        self._debug_printer.info('Net:' + str((end - start) / cv2.getTickFrequency()), 'fcnn')
        start = cv2.getTickCount()
        r, out_bin = cv2.threshold(out, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        tuple_candidates = VisionExtensions.findSpots(out_bin, self._pointcloud_stepsize, self._expand_stepsize, self._candidate_refinement_iteration_count)
        candidates = list()
        self._debug_printer.info(len(tuple_candidates), 'fcnn')
        for candidate in tuple_candidates:
            # calculate final width and height
            width, height = candidate[0] - candidate[1], candidate[3] - candidate[2]
            candidates.append(Candidate(candidate[1], candidate[2], width, height))
        end = cv2.getTickCount()
        self._debug_printer.info('Cluster:' + str((end - start) / cv2.getTickFrequency()), 'fcnn')
        return candidates

    def _get_raw_candidates(self):
        """
        returns a list of candidates [(Candidate), ...]
        :return: a list of candidates [(Candidate), ...]
        """
        out = self.get_fcnn_output()
        r, out_bin = cv2.threshold(out, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        candidates = list()
        # creating points
        # x shape
        xshape = self._image.shape[1]
        xlist = []
        x = 0
        while x < xshape:
            xlist.append(x)
            x += self._pointcloud_stepsize
        # y shape
        yshape = self._image.shape[0]
        ylist = []
        y = 0
        while y < yshape:
            ylist.append(y)
            y += self._pointcloud_stepsize
        # generate carthesian product of list
        points = list(itertools.product(xlist, ylist))
        if self._shuffle_candidate_list:
            random.shuffle(points)
        # expand points
        while points:
            point = points[-1]
            lx, uy = point
            rx, ly = point
            # expand to the left
            if not out_bin[point[1]][point[0]]:
                points.remove(point)
                continue
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
            for i in range(self._candidate_refinement_iteration_count):
                # expand from the middle of the borders of the found candidate
                width, height = rx - lx, ly - uy

                buffer_x = lx + width // 2
                buffer_y = uy + height // 2

                # expand to the left
                next_lx = max(lx - self._expand_stepsize, 0)
                while next_lx > 0 and out_bin[buffer_y][next_lx]:
                    lx = next_lx
                    next_lx = max(lx - self._expand_stepsize, 0)

                # expand to the right
                next_rx = min(rx + self._expand_stepsize, out_bin.shape[1] - 1)
                while next_rx < out_bin.shape[1] - 1 and out_bin[buffer_y][next_rx]:
                    rx = next_rx
                    next_rx = min(rx + self._expand_stepsize, out_bin.shape[1] - 1)

                # expand upwards
                next_uy = max(uy - self._expand_stepsize, 0)
                while next_uy > 0 and out_bin[next_uy][buffer_x]:
                    uy = next_uy
                    next_uy = max(uy - self._expand_stepsize, 0)

                # expand downwards
                next_ly = min(ly + self._expand_stepsize, out_bin.shape[0] - 1)
                while next_ly < out_bin.shape[0] - 1 and out_bin[next_ly][buffer_x]:
                    ly = next_ly
                    next_ly = min(ly + self._expand_stepsize, out_bin.shape[0] - 1)

            # calculate final width and height
            width, height = rx - lx, ly - uy
            candidates.append(Candidate(lx, uy, width, height))
            points.remove(point)
            points = [other_point for other_point in points if not (lx <= other_point[0] <= rx and uy <= other_point[1] <= ly)]
        return candidates

    def draw_debug_image(self):
        if not self._debug:
            return
        cv2.imshow('FCNN Output', self.get_fcnn_output())
        cv2.waitKey(1)

    def get_cropped_msg(self):
        msg = ImageWithRegionOfInterest()
        msg.header.frame_id = 'camera'
        msg.header.stamp = rospy.get_rostime()
        field_boundary_top = self._field_boundary_detector.get_upper_bound(y_offset=self._field_boundary_offset)
        image_cropped = self.get_fcnn_output()[field_boundary_top:]  # cut off at field_boundary
        msg.image = self.bridge.cv2_to_imgmsg(image_cropped, "mono8")
        msg.regionOfInterest.x_offset = 0
        msg.regionOfInterest.y_offset = field_boundary_top
        msg.regionOfInterest.height = self.get_fcnn_output().shape[0] - 1 - field_boundary_top
        msg.regionOfInterest.width = self.get_fcnn_output().shape[1] - 1
        return msg

