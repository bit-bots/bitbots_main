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
from bitbots_vision.vision_modules import ros_utils


class ColorDetector(object):
    """
    ColorDetector is abstract super-class of specialized sub-classes.
    ColorDetectors are used e.g. to check, if a pixel matches the defined color space
    or to create masked binary images.
    """

    def __init__(self, config):
        # type: (dict) -> None
        """
        Initialization of ColorDetector.

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        # Initial setup
        self.cv_bridge = CvBridge()

        self.config = {}
        self.update_config(config)

    def update_config(self, config):
        # type: (dict) -> None
        """
        Update (or initiate) the color detector setup with the new config.
        Always make a copy of self.config if a comparison between the old and new config is needed!

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        rospy.logdebug("(RE-)Configuring of ColorDetector")
        self.config = config

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
    def __init__(self, config, color_str, pub_hsv_mask_image=None):
        # type: (dict, str, rospy.Publisher) -> None
        """
        Initialization of HsvSpaceColorDetector.

        :param dict config: dictionary of the vision node configuration parameters
        :param str color_str: color (described in the config) that should be detected.
        :param rospy.Publisher pub_hsv_mask_image: Publisher of hsv mask images (Default: None)
        :return: None
        """
        self.detector_name = "{}_color_detector".format(color_str)
        self.pub_hsv_mask_image = pub_hsv_mask_image

        # Initialization of parent ColorDetector.
        super(HsvSpaceColorDetector, self).__init__(config)

    def update_config(self, config):
        # type: (dict) -> None
        """
        Update (or initiate) the color detector setup with the new config.
        Always make a copy of self.config if a comparison between the old and new config is needed!

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        super(HsvSpaceColorDetector, self).update_config(config)

        try:
            self.min_vals = np.array([
                        config[self.detector_name + '_lower_values_h'],
                        config[self.detector_name + '_lower_values_s'],
                        config[self.detector_name + '_lower_values_v']
                ])

            self.max_vals = np.array([
                        config[self.detector_name + '_upper_values_h'],
                        config[self.detector_name + '_upper_values_s'],
                        config[self.detector_name + '_upper_values_v']
                ])
        except KeyError:
            rospy.logerr("Undefined hsv color values for '{}'. Check config values.".format(self.detector_name))
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
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.min_vals, self.max_vals)

        # Toggle publishing of 'hsv_mask'-messages
        if self.pub_hsv_mask_image is not None:
            self.pub_hsv_mask_image.publish(self.cv_bridge.cv2_to_imgmsg(mask, '8UC1'))

        return mask


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

    def __init__(self, config, package_path, pub_field_mask_image=None):
        # type:(dict, str, rospy.Publisher) -> None
        """
        Initialization of PixelListColorDetector.

        :param dict config: dictionary of the vision node configuration parameters
        :param str package_path: path of package
        :param rospy.Publisher pub_field_mask_image: Publisher of field mask images (Default: None)
        :return: None
        """
        self.package_path = package_path
        self.pub_field_mask_image = pub_field_mask_image

        # Initialization of parent ColorDetector.
        super(PixelListColorDetector, self).__init__(config)

    def update_config(self, config):
        # type: (dict) -> None
        """
        Update (or initiate) the color detector setup with the new config.
        Always make a copy of self.config if a comparison between the old and new config is needed!

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        tmp_config = self.config.copy()

        super(PixelListColorDetector, self).update_config(config)

        # Toggle publishing of 'field_mask'-messages
        self.publish_field_mask_img_msg = config['vision_publish_field_mask_image']

        if ros_utils.config_param_change(tmp_config, config, [  'vision_use_sim_color',
                                                                'field_color_detector_path_sim',
                                                                'field_color_detector_path']):
            # concatenate path to file containing the accepted colors of base color space
            path = os.path.join(self.package_path, 'config/color_spaces')
            if config['vision_use_sim_color']:
                color_space_path = os.path.join(path, config['field_color_detector_path_sim'])
            else:
                color_space_path = os.path.join(path, config['field_color_detector_path'])

            self.color_space = self._init_color_space(color_space_path)

    def _init_color_space(self, color_path):
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

        # Toggle publishing of 'field_mask'-messages
        if self.pub_field_mask_image is not None and self.publish_field_mask_img_msg:
            self.pub_field_mask_image.publish(self.cv_bridge.cv2_to_imgmsg(mask, '8UC1'))

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

    def __init__(self, config, package_path, pub_field_mask_image=None, pub_dynamic_color_space_field_mask_image=None):
        # type:(dict, str, rospy.Publisher, rospy.Publisher) -> None
        """
        Initialization of DynamicPixelListColorDetector.

        :param dict config: dictionary of the vision node configuration parameters
        :param str package_path: path of package
        :param rospy.Publisher pub_field_mask_image: Publisher of field mask images (Default: None)
        :param rospy.Publisher pub_dynamic_color_space_field_mask_image: Publisher of dynamic color space field mask images (Default: None)
        :return: None
        """
        self.pub_dynamic_color_space_field_mask_image = pub_dynamic_color_space_field_mask_image

        # Initialization of parent PixelListColorDetector.
        super(DynamicPixelListColorDetector, self).__init__(config, package_path, pub_field_mask_image)

        global dyn_color_space
        dyn_color_space = np.copy(self.color_space)

        global base_color_space
        base_color_space = np.copy(self.color_space)

    def update_config(self, config):
        # type: (dict) -> None
        """
        Update (or initiate) the color detector setup with the new config.
        Always make a copy of self.config if a comparison between the old and new config is needed!

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        tmp_config = self.config.copy()

        super(DynamicPixelListColorDetector, self).update_config(config)

        # Toggle publishing of 'dynamic_field_mask'-messages
        if ros_utils.config_param_change(tmp_config, config, 'dynamic_color_space_publish_field_mask_image'):
            self.publish_dyn_field_mask_msg = self.config['dynamic_color_space_publish_field_mask_image']

    def mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Creates a color mask (0 for not in color range and 255 for in color range)
        and publishes static/dynamic field masks to 'ROS_field_mask_image_msg_topic' and 'ROS_dynamic_color_space_field_mask_image_msg_topic'.

        :param np.array image: image to mask
        :return np.array: masked image
        """
        global dyn_color_space
        dyn_mask = VisionExtensions.maskImg(image, dyn_color_space)

        if self.publish_field_mask_img_msg:
            global base_color_space
            static_mask = VisionExtensions.maskImg(image, base_color_space)

        # Toggle publishing of 'dynamic_field_mask'-messages
        if self.pub_dynamic_color_space_field_mask_image is not None and self.publish_dyn_field_mask_msg:
            self.pub_dynamic_color_space_field_mask_image.publish(self.cv_bridge.cv2_to_imgmsg(dyn_mask, '8UC1'))

        # Toggle publishing of 'field_mask'-messages
        if self.pub_field_mask_image is not None and self.publish_field_mask_img_msg:
            self.pub_field_mask_image.publish(self.cv_bridge.cv2_to_imgmsg(static_mask, '8UC1'))

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
        global base_color_space
        color_space_temp = np.copy(base_color_space)

        # Adds new colors to that color space
        color_space_temp[
            msg.blue,
            msg.green,
            msg.red] = 1

        # Switches the reference to the new color space
        global dyn_color_space
        dyn_color_space = color_space_temp
