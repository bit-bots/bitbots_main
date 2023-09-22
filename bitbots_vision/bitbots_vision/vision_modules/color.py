import os
import abc
import cv2
import yaml
import pickle
import numpy as np

from rclpy import logging
from cv_bridge import CvBridge
from bitbots_vision.vision_modules import ros_utils

logger = logging.get_logger('bitbots_vision')


class ColorDetector:
    """
    The abstract class :class:`.ColorDetector` defines a representation of valid colors e.g. the soccer field colors.
    It is used e.g. to check, if a pixel's color matches the defined color lookup table or to create masked binary images.
    As many of the modules rely on the color classification of pixels to generate their output, the color detector module matches their color to a given color lookup table.
    """

    def __init__(self, config):
        # type: (dict) -> None
        """
        Initialization of :class:`.ColorDetector`.

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        # Initial setup
        self._cv_bridge = CvBridge()

        self._image = None
        self._mask = None

        self._config = {}
        self.update_config(config)

        # Set if values should be cached
        self._caching = config['caching']

    def update_config(self, config):
        # type: (dict) -> None
        """
        Update (or initiate) the color detector setup with the new config.
        Always make a copy of self.config if a comparison between the old and new config is needed!

        :param dict config: dictionary of the vision node configuration parameters
        :return: None
        """
        logger.debug("(RE-)Configuring of ColorDetector")
        self._config = config

    @abc.abstractmethod
    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns, if bgr pixel is in color lookup table

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color lookup table or not
        """

    def set_image(self, image):
        # type: (np.array) -> None
        """
        Refreshes class variables after receiving an image

        :param image: the current frame of the video feed
        :return: None
        """
        self._image = image
        self._mask = None

    def get_mask_image(self, optional_image=None):
        # type: (np.array) -> np.array
        """
        Returns the color mask of the cached (or optional given) image
        (0 for not in color range and 255 for in color range)

        :param np.array optional_image: Optional input image
        :return np.array: masked image
        """
        if optional_image is not None:
            # Mask of optional image
            mask = self._mask_image(optional_image)
        else:
            # Mask of default cached image
            if self._mask is None or not self._caching:
                self._mask = self._mask_image(self._image)
            mask = self._mask

        return mask

    def get_normalized_image_mask(self, optional_image=None):
        # type: (np.array) -> np.array
        """
        Returns the image mask as described in `get_mask_image`, but the
        range of the values is one or zero and the dtype is a float.

        :param np.array optional_image: Optional input image
        :return np.array: masked image
        """
        return np.floor_divide(
            self.get_mask_image(optional_image),
            255, dtype=np.int16)

    @abc.abstractmethod
    def _mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Returns the color mask of the image
        (0 for not in color range and 255 for in color range)

        :param np.array image: input image
        :return np.array: masked image
        """

    def mask_bitwise(self, mask):
        # type: (np.array) -> np.array
        """
        Returns bitwise-and mask with current image

        :param np.array mask: mask
        :return np.array: bitwise-and mask with current image
        """
        return cv2.bitwise_and(self.get_mask_image(), self.get_mask_image(), mask=mask)

    def match_adjacent(self, image, point, offset=1, threshold=200):
        # type: (np.array, tuple[int, int], int, float) -> bool
        """
        Returns, if an area is in color lookup table

        :param np.array image: the full image
        :param tuple[int, int] point: a x-, y-tuple defining coordinates in the image
        :param int offset: the number of pixels to check in the surrounding of the
            point (like a radius but for a square)
        :param float threshold: the mean needed to accept the area to match (0-255)
        :return bool: whether area is in color lookup table or not
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
        Returns if an area is in color lookup table

        :param np.array area: the image area to check
        :param float threshold: the mean needed to accept the area to match (0-255)
        :return bool: whether area is in color lookup table or not
        """
        return np.mean(self.get_mask_image(area)) > threshold

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

    def compute(self):
        # type: () -> None
        """
        Compute image masks.

        :return: None
        """
        self.get_mask_image()


class HsvSpaceColorDetector(ColorDetector):
    """
    The :class:`.HsvSpaceColorDetector` is based on the HSV color space.
    The HSV color space is adjustable by setting min- and max-values for each hue, saturation and value.

    The values of the HSV channels can easily be adjusted by a human before a competition to match
    e.g. the white of the lines and goal or the team colors of the enemy team respectively.
    This is necessary as teams may have different tones of red or blue as their marker color.
    """

    def __init__(self, config, color_str):
        # type: (dict, str) -> None
        """
        Initialization of HsvSpaceColorDetector.

        :param dict config: dictionary of the vision node configuration parameters
        :param str color_str: color (described in the config) that should be detected.
        :return: None
        """
        self._detector_name = f"{color_str}_color_detector"

        # Hue channels span from 0 to 180 with a wrap around from 180 to 0. self._zero_crossing should be true, if the
        # used interval contains the zero-crossing and, hence, the wrap around at 180 to 0. Otherwise, false.
        try:
            self._zero_crossing = config[self._detector_name + "_h_zero_crossing"]
        except KeyError:
            logger.error(f"Undefined hsv detector values for '{self._detector_name}'. Check config values.")
            raise

        self._min_vals = None
        self._min_vals_A = None
        self._min_vals_B = None
        self._max_vals = None
        self._max_vals_A = None
        self._max_vals_B = None

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
            self._zero_crossing = config[self._detector_name + "_h_zero_crossing"]
            if self._zero_crossing:
                # If the zero-crossing lies in the interval, the interval is split into two intervals [0, x] and
                # [y, 180] with x, y in [0, 180]. A-values are for the [0, x] interval, B-values for the [y, 180]
                # interval.
                self._min_vals_A = np.array([
                    0,
                    config[self._detector_name + '_lower_values_s'],
                    config[self._detector_name + '_lower_values_v']
                ])

                self._max_vals_A = np.array([
                    config[self._detector_name + '_upper_values_h'],
                    config[self._detector_name + '_upper_values_s'],
                    config[self._detector_name + '_upper_values_v']
                ])

                self._min_vals_B = np.array([
                    config[self._detector_name + '_lower_values_h'],
                    config[self._detector_name + '_lower_values_s'],
                    config[self._detector_name + '_lower_values_v']
                ])

                self._max_vals_B = np.array([
                    180,
                    config[self._detector_name + '_upper_values_s'],
                    config[self._detector_name + '_upper_values_v']
                ])
            else:
                # If the zero-crossing is not in the interval, it is not decomposed and only one set of min and max
                # values is used.
                self._min_vals = np.array([
                    config[self._detector_name + '_lower_values_h'],
                    config[self._detector_name + '_lower_values_s'],
                    config[self._detector_name + '_lower_values_v']
                ])

                self._max_vals = np.array([
                    config[self._detector_name + '_upper_values_h'],
                    config[self._detector_name + '_upper_values_s'],
                    config[self._detector_name + '_upper_values_v']
                ])
        except KeyError:
            logger.error(f"Undefined hsv color values for '{self._detector_name}'. Check config values.")
            raise

    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns if bgr pixel is in color lookup table

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color lookup table or not
        """
        pixel = self.pixel_bgr2hsv(pixel)

        if self._zero_crossing:
            interval_A = (self._max_vals_A[0] >= pixel[0] >= self._min_vals_A[0]) and \
                         (self._max_vals_A[1] >= pixel[1] >= self._min_vals_A[1]) and \
                         (self._max_vals_A[2] >= pixel[2] >= self._min_vals_A[2])

            interval_B = (self._max_vals_B[0] >= pixel[0] >= self._min_vals_B[0]) and \
                         (self._max_vals_B[1] >= pixel[1] >= self._min_vals_B[1]) and \
                         (self._max_vals_B[2] >= pixel[2] >= self._min_vals_B[2])

            return interval_A or interval_B

        else:
            return (self._max_vals[0] >= pixel[0] >= self._min_vals[0]) and \
                (self._max_vals[1] >= pixel[1] >= self._min_vals[1]) and \
                (self._max_vals[2] >= pixel[2] >= self._min_vals[2])

    def _mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Returns the color mask of the image
        (0 for not in color range and 255 for in color range)

        :param np.array image: input image
        :return np.array: masked image
        """
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        if self._zero_crossing:
            interval_A = cv2.inRange(hsv_image, self._min_vals_A, self._max_vals_A)
            interval_B = cv2.inRange(hsv_image, self._min_vals_B, self._max_vals_B)
            return interval_A | interval_B
        else:
            return cv2.inRange(hsv_image, self._min_vals, self._max_vals)
