import cv2
from cv_bridge import CvBridge
import VisionExtensions
import numpy as np
from .candidate import Candidate, BallDetector
import itertools
import random
import rospy
from .live_fcnn_03 import FCNN03


class FcnnHandler(BallDetector):
    """
    This handles FCNNs, meaning it finds and rates candidates in their output.
    """

    def __init__(self, config, fcnn):
        """
        Inits the fcnn handler.
        :param dict config: dictionary of the vision node configuration parameters
        :param FCNN03 fcnn: a fcnn model
        """
        self._image = None
        self._fcnn = fcnn
        self._rated_candidates = None
        self._sorted_rated_candidates = None
        self._top_candidate = None
        self._fcnn_output = None
        self.bridge = CvBridge()

        # init config
        self.set_config(config)


    def set_image(self, image):
        """
        Set a image for the fcnn. This also resets the caches.
        :param image: current vision image
        :return: None
        """
        self._image = image
        self._rated_candidates = None
        self._sorted_rated_candidates = None
        self._top_candidate = None
        self._fcnn_output = None


    def set_config(self, config):
        """
        Set all configuration parameters for the fcnn.
        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        self._debug = config['ball_fcnn_publish_debug_img']
        self._threshold = config['ball_fcnn_threshold']
        self._expand_stepsize = config['ball_fcnn_expand_stepsize']
        self._pointcloud_stepsize = config['ball_fcnn_pointcloud_stepsize']
        self._min_candidate_diameter = config['ball_fcnn_min_ball_diameter']
        self._max_candidate_diameter = config['ball_fcnn_max_ball_diameter']
        self._candidate_refinement_iteration_count = config['ball_fcnn_candidate_refinement_iteration_count']


    def get_candidates(self):
        """
        Returns all ball candidates. This method is cached.
        :return: all ball candidates
        """
        # Check if a cached value exists
        if self._rated_candidates is None:
            # Create candidate list
            self._rated_candidates = list()
            # Run neural network and clustering and iterate over the given candidates
            for candidate in self._get_raw_candidates_cpp():
                # Get the fcnn heatmap
                out = self.get_fcnn_output()
                # Calculate the mean in the ROI in the heatmap
                candidate.rating = np.mean(
                    out[
                        candidate.get_upper_left_y():
                        candidate.get_upper_left_y() + candidate.get_height(),
                        candidate.get_upper_left_x():
                        candidate.get_upper_left_x() + candidate.get_width()]
                ) / 255.0
                # Check if candidate is in rating threshold and size bounds
                if self.inspect_candidate(candidate):
                    # Add candidate to list
                    self._rated_candidates.append(candidate)
        return self._rated_candidates

    def inspect_candidate(self, candidate):
        """
        Checks if candidates is in threshold. And in min/max diameter bounds.
        :param candidate: a Ball candidate
        :return: a boolean if the candidate satisfies these conditions
        """
        # type: (Candidate) -> bool
        return candidate.rating >= self._threshold \
               and self._min_candidate_diameter \
               <= candidate.get_diameter() \
               <= self._max_candidate_diameter

    def compute(self):
        """
        Runs the neural network.
        """
        # Call get candidates and drop the returned solution because it get cached for the real call
        self.get_candidates()

    def get_fcnn_output(self):
        """
        Calculates the fcnn heatmap. The output gets cached.
        :return: fcnn output
        """
        # Check if a cached one exists
        if self._fcnn_output is None:
            # Resize image for fcnn
            in_img = cv2.resize(self._image, (self._fcnn.input_shape[1], self._fcnn.input_shape[0]))
            # Convert image to floats
            in_img = cv2.cvtColor(in_img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            # Predict
            out = self._fcnn.predict(list([in_img]))
            # Reshape fcnn output
            out = out.reshape(self._fcnn.output_shape[0], self._fcnn.output_shape[1])
            # Convert back to uint8 dtype
            out = (out * 255).astype(np.uint8)
            # Resize the heatmap to match the resolution of the in comming image
            self._fcnn_output = cv2.resize(out, (self._image.shape[1], self._image.shape[0]))
        return self._fcnn_output

    def _get_raw_candidates_cpp(self):
        """
        Runs the fcnn heatmap clustering candidate detection.
        :return: ball candidates
        """
        start = cv2.getTickCount()
        # Get fcnn output
        out = self.get_fcnn_output()
        end = cv2.getTickCount()
        rospy.logdebug('Vision FCNN handler: Net:' + str((end - start) / cv2.getTickFrequency()))
        start = cv2.getTickCount()
        # Mask the heatmap with a threshold
        r, out_bin = cv2.threshold(out, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # Run cpp vision extention to find clusters in the heatmap
        tuple_candidates = VisionExtensions.findSpots(out_bin, self._pointcloud_stepsize, self._expand_stepsize, self._candidate_refinement_iteration_count)
        candidates = list()
        rospy.logdebug('Vision FCNN handler: ' + str(len(tuple_candidates)))
        # Convert output tuples to candidates
        for candidate in tuple_candidates:
            # Calculate final width and height
            width, height = candidate[0] - candidate[1], candidate[3] - candidate[2]
            candidates.append(Candidate(candidate[1], candidate[2], width, height))
        end = cv2.getTickCount()
        rospy.logdebug('Vision FCNN handler: Cluster:' + str((end - start) / cv2.getTickFrequency()))
        return candidates

    def get_debug_image(self):
        """
        Returns the fcnn heatmap as ros image message if debug is enabled.
        :return: fcnn heatmap
        """
        if self._debug:
            # Create image message with fcnn heatmap
            return self.bridge.cv2_to_imgmsg(self.get_fcnn_output(), "mono8")
