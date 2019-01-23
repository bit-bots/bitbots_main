# TODO: Debug, clean
import numpy as np
import VisionExtensions
from .debug import DebugPrinter
import yaml
import pickle
import time
import abc
import cv2
from collections import deque
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from bitbots_msgs.msg import Colorspace


class ColorDetector:
    def __init__(self, debug_printer):
        # type: (DebugPrinter) -> None
        """
        ColorDetector is abstract super-class of specialised sub-classes.
        ColorDetectors are used e.g. to check if a pixel matches the defined colorspace
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
        Returns if bgr pixel is in color space

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
        Returns if an area is in color space

        :param np.array image: the full image
        :param tuple[int, int] point: a x-, y-tuple defining coordinates in the image
        :param int offset: the number of pixels to check in the surounding of the
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
        HsvSpaceColorDetector is a ColorDetector, that is based on the HSV-colorspace.
        The HSV-colorspace is adjustable by setting min- and max-values for hue, saturation and value.

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
    def __init__(self, debug_printer, color_path):
        # type:(DebugPrinter, str) -> None
        """
        PixelListColorDetector is a ColorDetector, that is based on a colorspace.
        The colorspace is initially loaded from color_path and optionally adjustable to changing color-conditions.
        The colorspace is represented by boolean-values for RGB-color-values.

        :param DebugPrinter debug_printer: debug-printer
        :param str color_path: path to file containing the accepted colors
        :return: None
        """
        ColorDetector.__init__(self, debug_printer)
        self.color_space = self.init_color_space(color_path)
        self.base_color_space = np.copy(self.color_space)
        self.bridge = CvBridge()
        self.colorspace_sub = rospy.Subscriber('colorspace',
                                                Colorspace,
                                                self._colorspace_callback,
                                                queue_size=1,
                                                buff_size=2**20)
        # TODO remove or switch to debug pinter
        self.imagepublisher_dyn = rospy.Publisher("/mask_image_dyn", Image, queue_size=1)
        self.imagepublisher = rospy.Publisher("/mask_image", Image, queue_size=1)

    def _colorspace_callback(self, msg):
        self.decode_colorspace(msg)

    def decode_colorspace(self, msg):
        # Imports new colorspace from a ros msg. This is used to communicate with the Dynamic Colorspace Node.
        # Use the base colorspace as basis
        color_space_temp = np.copy(self.base_color_space)
        # Adds new colors to that colorspace
        color_space_temp[msg.blue,
                         msg.green,
                         msg.red] = 1
        # Switches the reference to the new colorspace
        self.color_space = color_space_temp
    
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
        Returns if bgr pixel is in color space

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color space or not
        """
        return self.color_space[pixel[0], pixel[1], pixel[2]]

    def mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Creates a color mask
        (0 for not in color range and 255 for in color range)

        :param np.array image: image to mask
        :return np.array: masked image
        """
        # TODO: remove or Debug
        mask = VisionExtensions.maskImg(image, self.color_space)
        mask_static = VisionExtensions.maskImg(image, self.base_color_space)
        self.imagepublisher_dyn.publish(self.bridge.cv2_to_imgmsg(mask, '8UC1'))
        self.imagepublisher.publish(self.bridge.cv2_to_imgmsg(mask_static, '8UC1'))

        return mask
