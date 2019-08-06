import abc
import cv2
import yaml
import pickle
import time
import rospy
import VisionExtensions
import numpy as np
import os
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from bitbots_msgs.msg import ColorSpace


class ColorDetector(object):
    """
    ColorDetector is abstract super-class of specialized sub-classes.
    ColorDetectors are used e.g. to check, if a pixel matches the defined color space
    or to create masked binary images.
    """

    def __init__(self):
        # type: () -> None
        """
        Initialization of ColorDetector.

        :return: None
        """
        pass

    @abc.abstractmethod
    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns, if bgr pixel is in color space

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color space or not
        """

    @abc.abstractmethod
    def mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Creates a color mask
        (0 for not in color range and 255 for in color range)

        :param np.array image: image to mask
        :return np.array: masked image
        """

    def match_adjacent(self, image, point, offset=1, threshold=200):
        # type: (np.array, tuple[int, int], int, float) -> bool
        """
        Returns, if an area is in color space

        :param np.array image: the full image
        :param tuple[int, int] point: a x-, y-tuple defining coordinates in the image
        :param int offset: the number of pixels to check in the surrounding of the
            point (like a radius but for a square)
        :param float threshold: the mean needed to accept the area to match (0-255)
        :return bool: whether area is in color space or not
        """
        area = image[
               max(0, point[1] - offset):
               min(image.shape[0] - 1, point[1] + offset),
               max(0, point[0] - offset):
               min(image.shape[1] - 1, point[0] + offset)
               ]
        return self.match_area(area, threshold=threshold)

    def match_area(self, area, threshold=200):
        # type: (np.array, float) -> bool
        """
        Returns if an area is in color space

        :param np.array area: the image area to check
        :param float threshold: the mean needed to accept the area to match (0-255)
        :return bool: whether area is in color space or not
        """
        return np.mean(self.mask_image(area)) > threshold

    @staticmethod
    def pixel_bgr2hsv(pixel):
        # type: (np.array) -> np.array
        """
        Converts bgr-pixel to hsv-pixel

        :param np.array pixel: brg-pixel
        :return np.array: hsv-pixel
        """
        pic = np.zeros((1, 1, 3), np.uint8)
        pic[0][0] = pixel
        return cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)[0][0]


class HsvSpaceColorDetector(ColorDetector):
    """
    HsvSpaceColorDetector is a ColorDetector, that is based on the HSV-color space.
    The HSV-color space is adjustable by setting min- and max-values for hue, saturation and value.
    """
    def __init__(self, config, color):
        # type: (dict, string) -> None
        """
        Initialization of HsvSpaceColorDetector.

        :param dict config: vision config
        :param string color: color (described in the config) that should be detected.
        :return: None
        """
        super(HsvSpaceColorDetector, self).__init__()

        detector_name = "{}_color_detector".format(color)

        try:
            self.min_vals = np.array([
                        config[detector_name + '_lower_values_h'],
                        config[detector_name + '_lower_values_s'],
                        config[detector_name + '_lower_values_v']
                ])

            self.max_vals = np.array([
                        config[detector_name + '_upper_values_h'],
                        config[detector_name + '_upper_values_s'],
                        config[detector_name + '_upper_values_v']
                ])
        except KeyError:
            rospy.logerr("Undefined hsv color detector for color '{}'. Check config values.".format(color))
            raise

    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns if bgr pixel is in color space

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color space or not
        """
        pixel = self.pixel_bgr2hsv(pixel)
        return (self.max_vals[0] >= pixel[0] >= self.min_vals[0]) and \
               (self.max_vals[1] >= pixel[1] >= self.min_vals[1]) and \
               (self.max_vals[2] >= pixel[2] >= self.min_vals[2])

    def mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Creates a color mask
        (0 for not in color range and 255 for in color range)

        :param np.array image: image to mask
        :return np.array: masked image
        """
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(image, self.min_vals, self.max_vals)


class PixelListColorDetector(ColorDetector):
    """
    PixelListColorDetector is a ColorDetector, that is based on a lookup table of color values.
    The color space is loaded from color-space-file at color_path (in config).
    The color space is represented by boolean-values for RGB-color-values.

    Publishes: 'ROS_field_mask_image_msg_topic'-messages

    The following parameters of the config dict are needed:
        'vision_use_sim_color',
        'field_color_detector_path_sim',
        'field_color_detector_path'
    """

    def __init__(self, package_path, config, pub_field_mask_image):
        # type:(str, dict, rospy.Publisher) -> None
        """
        Initialization of PixelListColorDetector.

        :param str package_path: path of package
        :param dict config: vision config
        :param rospy.Publisher pub_field_mask_image: Publisher of field mask images
        :return: None
        """
        super(PixelListColorDetector, self).__init__()
        self.bridge = CvBridge()

        self.config = config

        self.pub_field_mask_image = pub_field_mask_image

        # toggle publishing of mask_img msg
        self.publish_field_mask_img_msg = self.config['vision_publish_field_mask_image']

        color_space_path = os.path.join(package_path, 'config/color_spaces')
        # concatenate color-path to file containing the accepted colors of base color space
        if self.config['vision_use_sim_color']:
            self.color_path = os.path.join(color_space_path, self.config['field_color_detector_path_sim'])
        else:
            self.color_path = os.path.join(color_space_path, self.config['field_color_detector_path'])

        self.color_space = self.init_color_space(self.color_path)

    def init_color_space(self, color_path):
        # type: (str) -> None
        """
        Initialization of color space from yaml or pickle.txt file

        :param str color_path: path to file containing the accepted colors
        :return: None
        """
        color_space = np.zeros((256, 256, 256), dtype=np.uint8)
        if color_path.endswith('.yaml'):
            with open(color_path, 'r') as stream:
                try:
                    color_values = yaml.safe_load(stream)
                except yaml.YAMLError as exc:
                    rospy.logerr('Vision color detector: ' + exc)

        # pickle-file is stored as '.txt'
        elif color_path.endswith('.txt'):
            try:
                with open(color_path, 'rb') as f:
                    color_values = pickle.load(f)
            except pickle.PickleError as exc:
                rospy.logerr('Vision color detector: ' + exc)

        # compatibility with colorpicker
        if 'color_values' in color_values.keys():
            color_values = color_values['color_values']['greenField']
        length = len(color_values['red'])
        if length == len(color_values['green']) and \
                length == len(color_values['blue']):
            # setting colors from yaml file to True in color space
            for x in range(length):
                color_space[color_values['blue'][x], color_values['green'][x], color_values['red'][x]] = 1
        return color_space

    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns, if bgr pixel is in color space

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color space or not
        """
        return self.color_space[pixel[0], pixel[1], pixel[2]]

    def mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Creates a color mask (0 for not in color range and 255 for in color range)
        and publishes the field mask to 'ROS_field_mask_image_msg_topic'.

        :param np.array image: image to mask
        :return np.array: masked image
        """
        mask = VisionExtensions.maskImg(image, self.color_space)

        # toggle publishing of 'field_mask'-messages
        if self.publish_field_mask_img_msg]:
            self.pub_field_mask_image.publish(self.bridge.cv2_to_imgmsg(mask, '8UC1'))

        return mask


class DynamicPixelListColorDetector(PixelListColorDetector):
    """
    DynamicPixelListColorDetector is a ColorDetector, that is based on a lookup table of color values.
    The color space is initially loaded from color-space-file at color_path (in config)
    and optionally adjustable to changing color conditions (dynamic color space).
    The color space is represented by boolean-values for RGB-color-values.

    Subscribes to: 'ROS_dynamic_color_space_msg_topic'
    Publishes: 'ROS_field_mask_image_msg_topic' and
        'ROS_dynamic_color_space_field_mask_image_msg_topic'-messages
    """

    def __init__(self, package_path, config, pub_field_mask_image, pub_dynamic_color_space_field_mask_image, primary_detector=False):
        # type:(str, dict, rospy.Publisher, rospy.Publisher, bool) -> None
        """
        Initialization of DynamicPixelListColorDetector.

        :param str package_path: path of package
        :param dict config: vision config
        :param rospy.Publisher pub_field_mask_image: Publisher of field mask images
        :param rospy.Publisher pub_dynamic_color_space_field_mask_image: Publisher of dynamic color space field mask images
        :param bool primary_detector: true if it is the primary color detector
            (only the detector held by the vision node should be True) (Default: False)
            This allows publishing of field mask images.
        :return: None
        """
        super(DynamicPixelListColorDetector, self).__init__(package_path, config, pub_field_mask_image)

        self.pub_dynamic_color_space_field_mask_image = pub_dynamic_color_space_field_mask_image
        self.primary_detector = primary_detector

        self.base_color_space = np.copy(self.color_space)

        # toggle publishing of mask_img_dyn msg with dynamic color space
        self.publish_dyn_field_mask_msg = self.config['dynamic_color_space_publish_field_mask_image']

    def mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Creates a color mask (0 for not in color range and 255 for in color range)
        and publishes static/dynamic field masks to 'ROS_field_mask_image_msg_topic' and 'ROS_dynamic_color_space_field_mask_image_msg_topic'.

        :param np.array image: image to mask
        :return np.array: masked image
        """
        dyn_mask = VisionExtensions.maskImg(image, self.color_space)

        if self.publish_field_mask_img_msg:
            static_mask = VisionExtensions.maskImg(image, self.base_color_space)

        # toggle publishing of dynamic field masks
        if (self.primary_detector and self.publish_dyn_field_mask_msg):
            self.pub_dynamic_color_space_field_mask_image.publish(self.bridge.cv2_to_imgmsg(dyn_mask, '8UC1'))

        # toggle publishing of field masks
        if (self.primary_detector and self.publish_field_mask_img_msg):
            self.pub_field_mask_image.publish(self.bridge.cv2_to_imgmsg(static_mask, '8UC1'))

        return dyn_mask

    def color_space_callback(self, msg):
        # type: (ColorSpace) -> None
        """
        This callback gets called inside the vision node, after subscriber received ColorSpaceMessage from DynamicColorSpace-Node.

        :param ColorSpaceMessage msg: ColorSpaceMessage
        :return: None
        """
        self.decode_color_space(msg)

    def decode_color_space(self, msg):
        # type: (ColorSpaceMessage) -> None
        """
        Imports new color space from ros msg. This is used to communicate with the DynamicColorSpace-Node.

        :param ColorSpaceMessage msg: ColorSpaceMessage
        :return: None
        """
        # Create temporary color space
        # Use the base color space as basis
        color_space_temp = np.copy(self.base_color_space)

        # Adds new colors to that color space
        color_space_temp[
            msg.blue,
            msg.green,
            msg.red] = 1

        # Switches the reference to the new color space
        self.color_space = color_space_temp
