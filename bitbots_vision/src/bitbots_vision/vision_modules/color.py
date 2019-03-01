import abc
import cv2
import yaml
import pickle
import time
import rospy
import VisionExtensions
import numpy as np
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from bitbots_msgs.msg import ColorSpaceMessage
from .debug import DebugPrinter


class ColorDetector:
    def __init__(self, debug_printer):
        # type: (DebugPrinter) -> None
        """
        ColorDetector is abstract super-class of specialized sub-classes.
        ColorDetectors are used e.g. to check, if a pixel matches the defined color-space
        or to create masked binary-images.

        :param DebugPrinter debug_printer: debug-printer
        :return: None
        """
        self._debug_printer = debug_printer
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
    def __init__(self, debug_printer, min_vals, max_vals):
        # type: (DebugPrinter, tuple[int, int, int], tuple[int, int, int]) -> None
        """
        HsvSpaceColorDetector is a ColorDetector, that is based on the HSV-color-space.
        The HSV-color-space is adjustable by setting min- and max-values for hue, saturation and value.

        :param DebugPrinter debug_printer: debug-printer
        :param tuple min_vals: a tuple of the minimal accepted hsv-values
        :param tuple max_vals: a tuple of the maximal accepted hsv-values
        :return: None
        """
        ColorDetector.__init__(self, debug_printer)
        self.min_vals = np.array(min_vals)
        self.max_vals = np.array(max_vals)

    def set_config(self, min_vals, max_vals):
        # type: (tuple[int, int, int], tuple[int, int, int]) -> None
        """
        Updates the hsv-space configuration

        :param min_vals: a tuple of the minimal accepted hsv-values
        :param max_vals: a tuple of the maximal accepted hsv-values
        :return: None
        """
        self.min_vals = np.array(min_vals)
        self.max_vals = np.array(max_vals)

    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns if bgr pixel is in color space

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color space or not
        """
        pixel = self.pixel_bgr2hsv(pixel)
        # TODO: optimize comparisons
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

    # do not use this stuff!
    # def pixel_bgr2hsv(self, bgr_pixel):
    #     normalized_bgr_pixel = (bgr_pixel[0] / 255,
    #                             bgr_pixel[1] / 255,
    #                             bgr_pixel[2] / 255)
    #     min_bgr = min(normalized_bgr_pixel)
    #     index_max = max(xrange(len(bgr_pixel)), key=bgr_pixel.__getitem__)
    #
    #     # set V
    #     v = normalized_bgr_pixel[index_max]
    #     # set S
    #     s = 0
    #     if v is not 0:
    #         s = (v - min_bgr) / float(v)
    #     # set H
    #     buf = v - min_bgr
    #     if index_max is 0:
    #         h = 120 + 30 * (normalized_bgr_pixel[2] - normalized_bgr_pixel[1]) / buf
    #     elif index_max is 1:
    #         h = 60 + 30 * (normalized_bgr_pixel[0] - normalized_bgr_pixel[2]) / buf
    #     else:
    #         h = 30 * (normalized_bgr_pixel[1] - normalized_bgr_pixel[0]) / buf
    #     return tuple((int(h), int(s * 255), int(v * 255)))


class PixelListColorDetector(ColorDetector):
    def __init__(self, debug_printer, package_path, config, primary_detector=False):
        # type:(DebugPrinter, str, dict, bool) -> None
        """
        PixelListColorDetector is a ColorDetector, that is based on a color-space.
        The color-space is initially loaded from color-space-file at color_path (in config)
        and optionally adjustable to changing color-conditions (dynamic-color-space).
        The color-space is represented by boolean-values for RGB-color-values.

        :param DebugPrinter debug_printer: debug-printer
        :param str package_path: path of package
        :param dict config: vision-config
        :param bool primary_detector: is primary color-detector if True
            (only detector held by vision should be True) (Default: False)
        :return: None
        """
        ColorDetector.__init__(self, debug_printer)
        self.bridge = CvBridge()

        self.vision_config = config

        # TODO: debug-printer
        self.primary_detector = primary_detector

        # concatenate color-path to file containing the accepted colors of base-color-space
        self.color_path = package_path + self.vision_config['field_color_detector_path']
        self.base_color_space = self.init_color_space(self.color_path)
        self.color_space = np.copy(self.base_color_space)

        # toggle publishing of mask_img msg
        self.publish_mask_img_msg = self.vision_config['vision_mask_img_msg']
        
        # toggle publishing of mask_img_dyn msg with dynamic color-space
        self.publish_mask_img_dyn_msg = self.vision_config['vision_dynamic_color_space_mask_img_dyn_msg']

        # toggle use of dynamic-color-space
        self.is_dynamic_color_space = self.vision_config['vision_dynamic_color_space']

        # Subscribe to 'color_space'-messages from DynamicColorSpace
        self.color_space_subscriber = rospy.Subscriber(
            'color_space',
            ColorSpaceMessage,
            self.color_space_callback,
            queue_size=1,
            buff_size=2**20)
    
        # Register publisher for 'mask_image'-messages
        self.imagepublisher = rospy.Publisher(
            "/mask_image",
            Image,
            queue_size=1)

        # Register publisher for 'mask_image_dyn'-messages
        self.imagepublisher_dyn = rospy.Publisher(
            "/mask_image_dyn",
            Image,
            queue_size=1)

    def init_color_space(self, color_path):
        # type: (str) -> None
        """
        Initializes color space from yaml or pickle.txt file

        :param str color_path: path to file containing the accepted colors
        :return: None
        """
        color_space = np.zeros((256, 256, 256), dtype=np.uint8)
        if color_path.endswith('.yaml'):
            with open(color_path, 'r') as stream:
                try:
                    color_values = yaml.load(stream)
                except yaml.YAMLError as exc:
                    self._debug_printer.error(exc, 'PixelListColorDetector')
                    # TODO: what now??? Handle the error?

        # pickle-file is stored as '.txt'
        elif color_path.endswith('.txt'):
            try:
                with open(color_path, 'rb') as f:
                    color_values = pickle.load(f)
            except pickle.PickleError as exc:
                self._debug_printer.error(exc, 'PixelListColorDetector')
            
        # compatibility with colorpicker
        if 'color_values' in color_values.keys():
            color_values = color_values['color_values']['greenField']
        length = len(color_values['red'])
        if length == len(color_values['green']) and \
                        length == len(color_values['blue']):
            # setting colors from yaml file to True in color space
            for x in range(length):
                color_space[color_values['blue'][x],
                                color_values['green'][x],
                                color_values['red'][x]] = 1
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
        and publishes 'mask_img' and 'mask_img_dyn'-messages.

        :param np.array image: image to mask
        :return np.array: masked image
        """
        if (not self.is_dynamic_color_space) or self.publish_mask_img_msg:
            static_mask = VisionExtensions.maskImg(image, self.base_color_space)

        if self.is_dynamic_color_space:
            dyn_mask = VisionExtensions.maskImg(image, self.color_space)
            # toggle publishing of 'mask_img_dyn'-messages
            if (self.primary_detector and self.publish_mask_img_dyn_msg):
                self.imagepublisher_dyn.publish(self.bridge.cv2_to_imgmsg(dyn_mask, '8UC1'))
  
        # toggle publishing of 'mask_img'-messages          
        if (self.primary_detector and self.publish_mask_img_msg):
            self.imagepublisher.publish(self.bridge.cv2_to_imgmsg(static_mask, '8UC1'))

        if self.is_dynamic_color_space:
            return dyn_mask
        else:
            return static_mask

    def color_space_callback(self, msg):
        # type: (ColorSpace) -> None
        """
        This callback gets called, after subscriber received ColorSpaceMessage from DynamicColorSpace-Node.

        :param ColorSpaceMessage msg: ColorSpaceMessage
        :return: None
        """
        self.decode_color_space(msg)

    def decode_color_space(self, msg):
        # type: (ColorSpaceMessage) -> None
        """
        Imports new color-space from ros msg. This is used to communicate with the DynamicColorSpace-Node.

        :param ColorSpaceMessage msg: ColorSpaceMessage
        :return: None
        """
        # Create temporary color-space
        # Use the base-color-space as basis
        color_space_temp = np.copy(self.base_color_space)

        # Adds new colors to that color-space
        color_space_temp[
            msg.blue,
            msg.green,
            msg.red] = 1

        # Switches the reference to the new color-space
        self.color_space = color_space_temp
