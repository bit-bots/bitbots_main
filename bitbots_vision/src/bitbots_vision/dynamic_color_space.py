#! /usr/bin/env python3

import cv2
import yaml
import rospy
import rospkg
import numpy as np
from cv_bridge import CvBridge
from collections import deque
from sensor_msgs.msg import Image
from bitbots_msgs.msg import ColorSpace, Config
from bitbots_vision.vision_modules import field_boundary, color, debug, evaluator

class DynamicColorSpace:
    def __init__(self):
        # type: () -> None
        """
        DynamicColorSpace is a ROS node, that is used by the vision node to better recognize the field color.
        DynamicColorSpace is able to calculate dynamically changing color spaces to accommodate e.g. 
        changing lighting conditions or to compensate for not optimized base color space files.

        This node subscribes to an Image-message (default: image_raw) and to the 'vision_config'-message.
        This node publishes ColorSpace-messages.

        Initiating 'bitbots_dynamic_color_space' node.

        :return: None
        """
        # Init package
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_vision')

        rospy.init_node('bitbots_dynamic_color_space')
        rospy.loginfo('Initializing dynamic color-space...')

        self.bridge = CvBridge()

        # Init params
        self.vision_config = {}

        # Subscribe to 'vision_config'-message
        # The message topic name MUST be the same as in the config publisher in vision.py
        self.sub_vision_config_msg = rospy.Subscriber(
            'vision_config',
            Config,
            self.vision_config_callback,
            queue_size=1,
            tcp_nodelay=True)

        rospy.spin()

    def vision_config_callback(self, msg):
        # type: (Config) -> None
        """
        This method is called by the 'vision_config'-message subscriber.
        Load and update vision config.
        Handle config changes.

        This callback is delayed (about 30 seconds) after changes through dynamic reconfigure

        :param Config msg: new 'vision_config'-message subscriber
        :return: None
        """
        # Load dict from string in yaml-format in msg.data
        vision_config = yaml.load(msg.data)

        self.debug_printer = debug.DebugPrinter(
            debug_classes=debug.DebugPrinter.generate_debug_class_list_from_string(
                vision_config['vision_debug_printer_classes']))
        self.runtime_evaluator = evaluator.RuntimeEvaluator(None)

        # Print status of dynamic color space after toggeling 'dynamic_color_space_active' parameter
        if 'dynamic_color_space_active' not in self.vision_config or \
            vision_config['dynamic_color_space_active'] != self.vision_config['dynamic_color_space_active']:
            if vision_config['dynamic_color_space_active']:
                rospy.loginfo('Dynamic color space turned ON.')
            else:
                rospy.logwarn('Dynamic color space turned OFF.')

        # Set publisher of ColorSpace-messages
        if 'ROS_dynamic_color_space_msg_topic' not in self.vision_config or \
                self.vision_config['ROS_dynamic_color_space_msg_topic'] != vision_config['ROS_dynamic_color_space_msg_topic']:
            if hasattr(self, 'pub_color_space'):
                self.pub_color_space.unregister()
            self.pub_color_space = rospy.Publisher(
                vision_config['ROS_dynamic_color_space_msg_topic'],
                ColorSpace,
                queue_size=1)

        # Set Color- and FieldBoundaryDetector
        self.color_detector = color.DynamicPixelListColorDetector(
            self.debug_printer,
            self.package_path,
            vision_config)

        self.field_boundary_detector = field_boundary.FieldBoundaryDetector(
            self.color_detector,
            vision_config,
            self.debug_printer,
            used_by_dyn_color_detector=True)

        # Reset queue
        if hasattr(self, 'color_value_queue'):
            self.color_value_queue.clear()

        # Set params
        self.queue_max_size = vision_config['dynamic_color_space_queue_max_size']
        self.color_value_queue = deque(maxlen=self.queue_max_size)

        self.pointfinder = Pointfinder(
            self.debug_printer,
            vision_config['dynamic_color_space_threshold'],
            vision_config['dynamic_color_space_kernel_radius'])

        self.heuristic = Heuristic(self.debug_printer)

        # Subscribe to Image-message
        if 'ROS_img_msg_topic' not in self.vision_config or \
                self.vision_config['ROS_img_msg_topic'] != vision_config['ROS_img_msg_topic']:
            if hasattr(self, 'sub_image_msg'):
                self.sub_image_msg.unregister()
            self.sub_image_msg = rospy.Subscriber(
                vision_config['ROS_img_msg_topic'],
                Image,
                self.image_callback,
                queue_size=vision_config['ROS_img_queue_size'],
                tcp_nodelay=True,
                buff_size=60000000)
            # https://github.com/ros/ros_comm/issues/536

        self.vision_config = vision_config

    def image_callback(self, image_msg):
        # type: (Image) -> None
        """
        This method is called by the Image-message subscriber.
        Old Image-messages were dropped.

        Sometimes the queue gets to large, even when the size is limeted to 1. 
        That's, why we drop old images manually.

        :param Image image_msg: new Image-message from Image-message subscriber
        :return: None
        """
        # Turn off dynamic color space, if parameter of config is false
        if 'dynamic_color_space_active' not in self.vision_config or \
            not self.vision_config['dynamic_color_space_active']:
            return

        # Drops old images
        image_age = rospy.get_rostime() - image_msg.header.stamp 
        if image_age.to_sec() > 0.1:
            self.debug_printer.info('Dynamic color space: Dropped Image-message', 'image')
            return

        self.handle_image(image_msg)

    def handle_image(self, image_msg):
        # type: (Image) -> None
        """
        This method handles the processing of an Image-message.
        New colors are calculated, appended to queue and published.

        :param Image image_msg: Image-message
        :return: None
        """
        # Converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        # Get new dynamic colors from image
        colors = self.get_new_dynamic_colors(image)
        # Add new colors to the queue
        self.color_value_queue.append(colors)
        # Publishes to 'ROS_dynamic_color_space_msg_topic'
        self.publish(image_msg)

    def get_unique_color_values(self, image, coordinate_list):
        # type: (np.array, np.array) -> np.array
        """
        Returns array of unique colors values from a given image at pixel coordinates from given list.

        :param np.array image: image
        :param np.array coordinate_list: list of pixel coordinates
        :return np.array: array of unique color values from image at pixel coordinates
        """
        # Create list of color values from image at given coordinates
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
        Returns array of new dynamically calculated color values.
        Those values were filtered by the heuristic.

        :param np.array image: image
        :return np.array: array of new dynamic color values
        """
        # Masks new image with current color space
        mask_image = self.color_detector.mask_image(image)
        # Get mask from field_boundary detector
        self.field_boundary_detector.set_image(image)
        mask = self.field_boundary_detector.get_mask()
        if mask is not None:
            # Get array of pixel coordinates of color candidates
            pixel_coordinates = self.pointfinder.get_coordinates_of_color_candidates(mask_image)
            # Get unique color values from the candidate pixels
            color_candidates = self.get_unique_color_values(image, pixel_coordinates)
            # Filters the colors using the heuristic.
            colors = np.array(self.heuristic.run(color_candidates, image, mask), dtype=np.int32)
            return colors
        return np.array([[]])

    def queue_to_color_space(self, queue):
        # type: (deque) -> np.array
        """
        Returns color space as array of all queue elements stacked, which contains all colors from the queue.

        :param dequeue queue: queue of array of color values
        :return np.array: color space
        """
        # Initializes an empty color space
        color_space = np.array([]).reshape(0,3)
        # Stack every color space in the queue
        for new_color_value_list in queue:
            color_space = np.append(color_space, new_color_value_list[:,:], axis=0)
        # Return a color space, which contains all colors from the queue
        return color_space

    def publish(self, image_msg):
        # type: (Image) -> None
        """
        Publishes the current color space via ColorSpace-message.

        :param Image image_msg: 'image_raw'-message
        :return: None
        """
        # Get color space from queue
        color_space = self.queue_to_color_space(self.color_value_queue)
        # Create ColorSpace-message
        color_space_msg = ColorSpace()
        color_space_msg.header.frame_id = image_msg.header.frame_id
        color_space_msg.header.stamp = image_msg.header.stamp
        color_space_msg.blue  = color_space[:,0].tolist()
        color_space_msg.green = color_space[:,1].tolist()
        color_space_msg.red   = color_space[:,2].tolist()
        # Publish ColorSpace-message
        self.pub_color_space.publish(color_space_msg)


class Pointfinder():
    def __init__(self, debug_printer, threshold, kernel_radius):
        # type: (DebugPrinter, float, int) -> None
        """
        Pointfinder is used to find false-color pixels with higher true-color / false-color ratio as threshold in their surrounding in masked image.

        :param DebugPrinter: debug-printer
        :param float threshold: necessary amount of previously detected color in percentage
        :param int kernel_radius: radius surrounding the center element of kernel matrix, defines relevant surrounding of pixel
        :return: None
        """
        # Init params
        self.debug_printer = debug_printer

        self.threshold = threshold

        self.kernel_radius = kernel_radius
        self.kernel_edge_size = 2 * self.kernel_radius + 1

        # Defines kernel
        # Init kernel as M x M matrix of ONEs
        self.kernel = None
        self.kernel = np.ones((self.kernel_edge_size, self.kernel_edge_size))
        # Set value of the center element of the matrix to the negative of the count of the matrix elements
        # In case the value of this pixel is 1, it's value in the sum_array would be 0
        self.kernel[int(np.size(self.kernel, 0) / 2), int(np.size(self.kernel, 1) / 2)] = - self.kernel.size

    def get_coordinates_of_color_candidates(self, masked_image):
        # type (np.array) -> np.array
        """
        Returns array of pixel coordinates of color candidates.
        Color candidates are false-color pixels with a higher true-color/ false-color ratio as threshold in their surrounding in masked image.

        :param np.array masked_image: masked image
        :return np.array: list of indices
        """
        # Normalizes the masked image to values of 1 or 0
        normalized_image = np.floor_divide(masked_image, 255, dtype=np.int16)

        # Calculates the count of neighbors for each pixel
        sum_array = cv2.filter2D(normalized_image, -1, self.kernel, borderType=0)
        # Returns all pixels with a higher true-color / false-color ratio than the threshold
        return np.array(np.where(sum_array > self.threshold * (self.kernel.size - 1)))

class Heuristic:
    def __init__(self, debug_printer):
        # type: (DebugPrinter) -> None
        """
        Filters new color space colors according to their position relative to the field boundary.
        Only colors that occur under the field boundary and have no occurrences over the field boundary get picked.

        :param DebugPrinter debug_printer: Debug-printer
        :return: None
        """
        self.debug_printer = debug_printer

    def run(self, color_list, image, mask):
        # type: (np.array, np.array, np.array) -> np.array
        """
        This method filters a given list of colors using the original image and a field-boundary-mask.

        :param np.array color_list: list of color values, that need to be filtered
        :param np.array image: raw vision image
        :param np.array mask: binary field-boundary-mask
        :return np.array: filtered list of colors
        """
        # Simplifies the handling by merging the three color channels
        color_list = self.serialize(color_list)
        # Making a set and removing duplicated colors
        color_set = set(color_list)
        # Generates whitelist
        whitelist = self.recalculate(image, mask)
        # Takes only whitelisted values 
        color_set = color_set.intersection(whitelist)
        # Restructures the color channels
        return self.deserialize(np.array(list(color_set)))

    def recalculate(self, image, mask):
        # type: (np.array, np.array) -> set
        """
        Generates a whitelist of allowed colors using the original image and the field-boundary-mask.

        :param np.array image: image
        :param np.array mask: field-boundary-mask
        :return set: whitelist
        """
        # Generates whitelist
        colors_over_field_boundary, colors_under_field_boundary = self.unique_colors_for_partitions(image, mask)
        return set(colors_under_field_boundary) - set(colors_over_field_boundary)

    def unique_colors_for_partitions(self, image, mask):
        # type: (np.array, np.array) -> (np.array, np.array)
        """
        Masks picture and returns the unique colors that occurs in both mask partitions.

        :param np.array image: image
        :param np.array mask: field-boundary-mask
        :return np.array: colors over the field boundary
        :return np.array: colors under the field boundary
        """
        # Calls a function to calculate the number of occurrences of all colors in the image
        return (self.get_unique_colors(cv2.bitwise_and(image, image, mask=255 - mask)),
                self.get_unique_colors(cv2.bitwise_and(image, image, mask=mask)))

    def get_unique_colors(self, image):
        # type: (np.array) -> np.array
        """
        Calculates unique color for an input image.

        :param np.array image: image
        :return np.array: unique colors 
        """
        # Simplifies the handling by merging the 3 color channels
        serialized_img = self.serialize(np.reshape(image, (1, int(image.size / 3), 3))[0])
        # Returns unique colors in the image
        return np.unique(serialized_img, axis=0)

    def serialize(self, input_matrix):
        # type: (np.array) -> np.array
        """
        Serializes the different color channels of a list of colors in an single channel. (Like a HTML color code)

        :param np.array input_matrix: list of colors values with 3 channels
        :return: list of serialized colors
        """
        return np.array(
            np.multiply(input_matrix[:, 0], 256 ** 2) \
            + np.multiply(input_matrix[:, 1], 256) \
            + input_matrix[:, 2], dtype=np.int32)

    def deserialize(self, input_matrix):
        # type: (np.array) -> np.array
        """
        Resolves the serialization of colors into different channels. (Like a HTML color code)

        :param np.array input_matrix: serialized colors
        :return: original colors with 3 channels
        """
        new_matrix = np.zeros((input_matrix.size, 3))
        new_matrix[:, 0] = np.array(input_matrix // 256 ** 2, dtype=np.uint8)
        new_matrix[:, 1] = np.array((input_matrix - (new_matrix[:, 0] * 256 ** 2)) / 256, dtype=np.uint8)
        new_matrix[:, 2] = np.array((input_matrix - (new_matrix[:, 0] * 256 ** 2) - (new_matrix[:, 1] * 256)), dtype=np.uint8)
        return new_matrix


if __name__ == '__main__':
    DynamicColorSpace()
