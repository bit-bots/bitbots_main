import cv2
from cv_bridge import CvBridge
from humanoid_league_msgs.msg import ImageWithRegionOfInterest
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

    Example configuration (put these values into the config dictionary or just
    copy them into the vision package config):

    ball_fcnn: TODO fcnn config improvement
        publish_debug_img: false  # toggles publishing of the fcnn heatmap image for debug purposes
        model_path: '/models/2019_05_timon_basler'
        threshold: .6  # minimal value for a candidate to be considered
        expand_stepsize: 4
        pointcloud_stepsize: 10
        shuffle_candidate_list: false  # shuffles the list of possible candidate points
        min_ball_diameter: 15
        max_ball_diameter: 150
        publish_output: false
        publish_field_boundary_offset: 5
    """

    def __init__(self, fcnn, field_boundary_detector, config):
        """
        Inits the fcnn handler.
        :param fcnn: a fcnn model
        :param config: TODO
        """
        self._image = None
        self._fcnn = fcnn
        self._field_boundary_detector = field_boundary_detector
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
        """
        self._image = image
        self._rated_candidates = None
        self._sorted_rated_candidates = None
        self._top_candidate = None
        self._fcnn_output = None


    def set_config(self, config):
        # TODO new
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

    def get_top_candidates(self, count=1):
        """
        Returns the 'count' best candidates.
        :param count: Number of top-candidates to return
        :return: the count top candidates
        """
        if count < 1:
            raise ValueError('the count must be equal or greater 1!')
        # Check if cached
        if self._sorted_rated_candidates is None:
            # Sort candidates by rating
            self._sorted_rated_candidates = sorted(self.get_candidates(), key=lambda x: x.get_rating())
        return self._sorted_rated_candidates[0:count]

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

    def get_cropped_msg(self):
        """
        Returns a region of interest with the fcnn heatmap under the field boundary.
        :return: fcnn heatmap under the field boundary
        """
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

