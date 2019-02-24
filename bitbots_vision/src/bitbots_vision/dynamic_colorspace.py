#! /usr/bin/env python2

import cv2
import time
import yaml
import rospy
import rospkg
import numpy as np
from cv_bridge import CvBridge
from collections import deque
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from sensor_msgs.msg import Image
from bitbots_msgs.msg import Colorspace, Config
from bitbots_vision.vision_modules import debug
from bitbots_vision.vision_modules import horizon, color
from bitbots_vision.cfg import dynamic_colorspaceConfig

# 
# :return TODO rename toggle in color.py
# 
# :return TODO debug_printer usage in color.py
# TODO better parameter-names in config
# TODO remove dyn from launch file
# TODO vision-docu
# TODO vision: set debug_printer as first param?
# TODO find_colorpixel_candidates what do we return?
# TODO change order of methods
# TODO docu heuristic
# TODO todos in cfgs and yamls
# TODO refactor everything to color-space
# TODO do class -methods, -variables with _underscore?
# TODO rename Config-Message

# TODO register image_raw subscriber for vision.py in init?

class DynamicColorspace:
    def __init__(self):
        # type: () -> None
        """
        DynamicColorspace is a ROS node, that is used by the vision-node to better recognize the field-color.
        DynamicColorspace is able to calculate dynamically changing colorspaces to accommodate e.g. 
        changing lighting conditions or to compensate for not optimized base-colorspace-files.

        Initiating DynamicColorspace-node.

        :return: None
        """
        # Init package
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path('bitbots_vision')

        rospy.init_node('bitbots_dynamic_colorspace')
        rospy.loginfo('Initializing dynamic colorspace...')

        self._bridge = CvBridge()

        # Load config
        self._config = {}
        self._config = rospy.get_param('/bitbots_dynamic_colorspace')

        # Load vision-config
        self._vision_config = {}
        # TODO maybe get config from msg???TODO docu
        self._vision_config = rospy.get_param('/bitbots_vision')

        # TODO NOT tested... correct implementation?
        self._debug_printer = debug.DebugPrinter(
            debug_classes=debug.DebugPrinter.generate_debug_class_list_from_string(
                self._config['dynamic_color_space_debug_printer_classes']))

        # Init params
        self._queue_max_size = self._config['queue_max_size']
        self._color_value_queue = deque(maxlen=self._queue_max_size)

        self._pointfinder_threshold = self._config['threshold']
        self._pointfinder_kernel_size = self._config['kernel_size']

        # Set ColorDetector and HorizonDetector
        self._set_detectors()

        self._pointfinder = Pointfinder(
            self._debug_printer,
            self._pointfinder_threshold,
            self._pointfinder_kernel_size)

        self._heuristic = Heuristic(self._debug_printer)

        # Subscribe to 'image-raw'-message
        self._image_raw_msg_subscriber = rospy.Subscriber(
            'image_raw',
            Image,
            self._image_callback,
            queue_size=1,
            tcp_nodelay=True,
            buff_size=60000000)

        # Subscribe to 'vision_config'-message
        self._vision_config__msg_subscriber = rospy.Subscriber(
            'vision_config',
            Config,
            self._vision_config_callback,
            queue_size=1,
            tcp_nodelay=True)

        # Register publisher of 'colorspace'-messages
        self._colorspace_publisher = rospy.Publisher(
            'colorspace',
            Colorspace,
            queue_size=1)

        # Register dynamic-reconfigure config-callback
        Server(dynamic_colorspaceConfig, self._dynamic_reconfigure_callback)

        rospy.spin()

    def _set_detectors(self):
        # type: () -> None
        """
        (Re-)Set Color- and HorizonDetector to newest vision-config.

        :return: None
        """
        self._color_detector = color.PixelListColorDetector(
            self._debug_printer,
            self._package_path,
            self._vision_config,
            do_publish_mask_img_msg=False)
            
        self._horizon_detector = horizon.HorizonDetector(
            self._color_detector,
            self._vision_config,
            self._debug_printer)

    def _dynamic_reconfigure_callback(self, config, level):
        # type: (dict, int) -> dict
        """
        This gets called, after dynamic-reconfigure-server received config-changes.
        Handling all config-changes.

        :param dict config: new config
        :param int level: ???
        :return dict: new config
        """
        # Set new config
        self._config = config

        self._set_detectors()

        # Reset queue
        self._color_value_queue.clear()

        # Set queue to new max-size
        self._queue_max_size = self._config['queue_max_size']
        self._color_value_queue = deque(maxlen=self._queue_max_size)
        
        # Set new config for Pointfinder
        self._threshold = self._config['threshold']
        self._kernel_size = self._config['kernel_size']
        self._pointfinder.set_pointfinder_params(self._threshold, self._kernel_size)

        return config

    def _vision_config_callback(self, msg):
        # type: (Config) -> None
        """
        This method is called by the 'vision_config'-message-subscriber.
        Load and update vision-config.
        Handle config-changes.

        :param Config msg: new 'vision_config'-message-subscriber
        :return: None
        """
        # Load dict from yaml-string in msg.data
        self._vision_config = yaml.load(msg.data)

        self._set_detectors()

    def _image_callback(self, image_msg):
        # type: (Image) -> None
        """
        This method is called by the 'img_raw'-message-subscriber.
        This handles Image-messages and drops old ones.

        Sometimes the queue gets to large, even when the size is limeted to 1. 
        That's, why we drop old images manualy.

        :param Image image_msg: new image-message from 'img_raw'-message-subscriber
        :return: None
        """
        # Turn off dynamic-color-space
        if not self._vision_config['vision_dynamic_colorspace']:
            return

        # Drops old images
        # TODO: debug_printer usage
        image_age = rospy.get_rostime() - image_msg.header.stamp 
        if image_age.to_sec() > 0.1:
            return

        # Converting the ROS image message to CV2-image
        image = self._bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        # Get new dynamic-colors from image
        colors = self.get_new_dynamic_colors(image)
        # Add new colors to the queue
        self._color_value_queue.append(colors)
        # Publishes the colorspace
        self.publish(image_msg)

    def get_unique_color_values(self, image, coordinate_list):
        # type: (np.array, np.array) -> np.array
        """
        Returns array of unique colors-values from a given image at pixel-coordinates from given list.

        :param np.array image: image
        :param np.array coordinate_list: list of pixel-coordinates
        :return np.array: array of unique color-values from image at pixel-coordinates
        """
        # Create list of color-values from image at given coordinates
        colors = image[coordinate_list[0], coordinate_list[1]]
        # np.unique requires list with at least one element
        if colors.size > 0:
            unique_colors = np.unique(colors, axis=0)
        else:
            unique_colors = colors
        return unique_colors

    def get_new_dynamic_colors(self, image):
        # type: (np.array) -> np.array
        """
        Returns array of new dynamically calculated color-values.
        Those values were filtered by the heuristic.

        :param np.array image: image
        :return np.array: array of new dynamic-color-values
        """
        # Masks new image with current colorspace
        mask_image = self._color_detector.mask_image(image)
        # Get mask from horizon detector
        self._horizon_detector.set_image(image)
        self._horizon_detector.compute_horizon_points()
        mask = self._horizon_detector.get_mask()
        if not mask is None:
            # Get array of pixel-coordinates of color-candidates
            pixel_coordinates = self._pointfinder.get_coordinates_of_color_candidates(mask_image)
            # Get unique color-values from the candidate pixels
            color_candidates = self.get_unique_color_values(image, pixel_coordinates)
            # Filters the colors using the heuristic.
            colors = np.array(self._heuristic.run(color_candidates, image, mask), dtype=np.int32)
            return colors
        return np.array([[]])

    def queue_to_colorspace(self, queue):
        # type: (dequeue) -> np.array
        """
        Returns colorspace as array of all queue-elements stacked, which contains all colors from the queue.

        :param dequeue queue: queue of array of color-values
        :return np.array: colorspace
        """
        # Initializes an empty colorspace
        colorspace = np.array([]).reshape(0,3)
        # Stack every colorspace in the queue
        for new_color_value_list in queue:
            colorspace = np.append(colorspace, new_color_value_list[:,:], axis=0)
        # Return a colorspace, which contains all colors from the queue
        return colorspace

    def publish(self, image_msg):
        # type: (Image) -> None
        """
        Publishes the current colorspace via Colorspace-message.

        :param Image image_msg: 'image_raw'-message
        :return: None
        """
        # Get colorspace from queue
        colorspace = self.queue_to_colorspace(self._color_value_queue)
        # Create Colorspace-message
        colorspace_msg = Colorspace()
        colorspace_msg.header.frame_id = image_msg.header.frame_id
        colorspace_msg.header.stamp = image_msg.header.stamp
        colorspace_msg.blue  = colorspace[:,0].tolist()
        colorspace_msg.green = colorspace[:,1].tolist()
        colorspace_msg.red   = colorspace[:,2].tolist()
        # Publish Colorspace-message
        self._colorspace_publisher.publish(colorspace_msg)


class Pointfinder():
    def __init__(self, debug_printer, threshold, kernel_size=3):
        # type: (DebugPrinter, float, int) -> None
        """
        Pointfinder is used to find false-color pixels with higher true-color / false-color ratio as threshold in their surrounding in masked-image.

        :param DebugPrinter: debug-printer
        :param float threshold: necessary amount of true-color in percentage (between 0..1)
        :param int kernel_size: edge-size of convolution kernel, use odd number (default 3), defines relevant surrounding of pixel
        :return: None
        """
        # Init params
        self._debug_printer = debug_printer
        self.set_pointfinder_params(threshold, kernel_size)

    def set_pointfinder_params(self, threshold, kernel_size=3):
        # type: (float, int) -> None
        """
        Set parameter of Pointfinder.
        Used to update parameter in case of dynamic reconfigure-config-changes.

        :param float threshold: necessary amount of true color in percentage (between 0..1)
        :param int kernel_size: defines edge-size of convolution kernel, use odd number (default 3)
        :return: None
        """
        self._threshold = threshold

        # Defines kernel
        # Init kernel as M x M matrix of ONEs
        self._kernel = None
        self._kernel = np.ones((kernel_size, kernel_size))
        # Set value of element in the middle of the matrix to negative of the count of the matrix-elements
        self._kernel[int(np.size(self._kernel, 0) / 2), int(np.size(self._kernel, 1) / 2)] = - self._kernel.size

    def get_coordinates_of_color_candidates(self, masked_image):
        # type (np.array) -> np.array
        """
        Returns array of pixel-coordinates of color-candidates.
        Color-candidates are false-color pixels with a higher true-color/ false-color ratio as threshold in their surrounding in masked-image.

        :param np.array masked_image: masked image
        :return np.array: list of indices
        """
        # Normalizes the masked image to values of 1 or 0
        normalized_image = np.divide(masked_image, 255, dtype=np.int16)

        # Calculates the count of neighbors for each pixel
        sum_array = cv2.filter2D(normalized_image, -1, self._kernel, borderType=0)
        # Returns all pixels with a higher true-color / false-color ratio than the threshold
        return np.array(np.where(sum_array > self._threshold * (self._kernel.size - 1)))

class Heuristic:
    def __init__(self, debug_printer):
        # type: (DebugPrinter) -> None
        """
        TODO

        :param DebugPrinter debug_printer: Debug-printer
        :return: None
        """
        self._debug_printer = debug_printer

    def run(self, color_list, image, mask):
        # type: (np.array, np.array, np.array) -> TODO
        """
        TODO

        :param np.array color_list: list of color-values
        :param np.array image: image
        :param np.array mask: horizon-mask
        :return: TODO
        """
        # Simplifies the handling by merging the three color-channels
        color_list = self.serialize(color_list)
        color_set = set(color_list)
        # Generates whitelist
        whitelist = self.recalc(image, mask)
        # Takes only whitelisted values 
        color_set = color_set.intersection(whitelist)
        # Restructures the color channels
        return self.deserialize(np.array(list(color_set)))

    def recalc(self, image, mask):
        # type: (np.array, np.array) -> TODO
        """
        TODO

        :param np.array image: image
        :param np.array mask: horizon-mask
        :return: TODO
        """
        # Generates whitelist
        colors_over_horizon, colors_under_horizon = self.calc_freq(image, mask)
        return set(colors_under_horizon) - set(colors_over_horizon)

    def calc_freq(self, image, mask):
        # type: (np.array, np.array) -> TODO
        """
        TODO

        :param np.array image: image
        :param np.array mask: horizon-mask
        :return: TODO
        """
        # Calls a function to calculate the number of occurrences of all colors in the image
        # returns array(over_horizon(unique, count), under_horizon(unique, count))
        return (self.get_freq_colors(cv2.bitwise_and(image, image, mask=255 - mask)),
                self.get_freq_colors(cv2.bitwise_and(image, image, mask=mask)))

    def get_freq_colors(self, image):
        # type: (np.array) -> TODO
        """
        TODO

        :param np.array image: image
        :return: TODO
        """
        # Simplifies the handling by merging the 3 color channels
        serialized_img = self.serialize(np.reshape(image, (1, int(image.size / 3), 3))[0])
        # Returns unique colors in the image
        return np.unique(serialized_img, axis=0)

    def serialize(self, input_matrix):
        # type: (TODO) -> TODO
        """
        TODO

        :param TODO input_matrix: TODO
        :return: TODO
        """
        return np.array(
            np.multiply(input_matrix[:, 0], 256 ** 2) \
            + np.multiply(input_matrix[:, 1], 256) \
            + input_matrix[:, 2], dtype=np.int32)

    def deserialize(self, input_matrix):
        # type: (TODO) -> TODO
        """
        TODO

        :param TODO input_matrix: TODO
        :return: TODO
        """
        new_matrix = np.zeros((input_matrix.size, 3))
        new_matrix[:, 0] = np.array(input_matrix // 256 ** 2, dtype=np.uint8)
        new_matrix[:, 1] = np.array((input_matrix - (new_matrix[:, 0] * 256 ** 2)) / 256, dtype=np.uint8)
        new_matrix[:, 2] = np.array((input_matrix - (new_matrix[:, 0] * 256 ** 2) - (new_matrix[:, 1] * 256)), dtype=np.uint8)
        return new_matrix


if __name__ == '__main__':
    DynamicColorspace()