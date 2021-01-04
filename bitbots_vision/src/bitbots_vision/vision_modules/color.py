import abc
import cv2
import yaml
import pickle
import rospy
import VisionExtensions
import numpy as np
import os
from cv_bridge import CvBridge
from bitbots_vision.vision_modules import ros_utils


class ColorDetector(object):
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
        rospy.logdebug("(RE-)Configuring of ColorDetector", logger_name="vision_color_detector")
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
            rospy.logerr(f"Undefined hsv color values for '{self._detector_name}'. Check config values.", logger_name="vision_hsv_color_detector")
            raise

    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns if bgr pixel is in color lookup table

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color lookup table or not
        """
        pixel = self.pixel_bgr2hsv(pixel)
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
        return cv2.inRange(hsv_image, self._min_vals, self._max_vals)


class PixelListColorDetector(ColorDetector):
    """
    The :class:`.PixelListColorDetector` is based on a lookup table of color values.
    The color lookup table is loaded from color-lookup-table-file defined in config.
    """

    def __init__(self, config, package_path, color_lookup_table_path_param='field_color_detector_path'):
        # type:(dict, str) -> None
        """
        Initialization of PixelListColorDetector.

        :param dict config: dictionary of the vision node configuration parameters
        :param str package_path: path of package
        :return: None
        """
        self._package_path = package_path

        self._color_lookup_table_path_param = color_lookup_table_path_param

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
        tmp_config = self._config.copy()

        super(PixelListColorDetector, self).update_config(config)

        if ros_utils.config_param_change(tmp_config, config, self._color_lookup_table_path_param):
            # concatenate path to file containing the accepted colors of base color lookup table
            path = os.path.join(self._package_path, 'config', 'color_lookup_tables')
            color_lookup_table_path = os.path.join(path, config[self._color_lookup_table_path_param])
            self._color_lookup_table = self._init_color_lookup_table(color_lookup_table_path)

    def _init_color_lookup_table(self, color_path):
        # type: (str) -> None
        """
        Initialization of color lookup table from .yaml or .pickle file

        :param str color_path: path to file containing the accepted colors
        :return: None
        """
        color_lookup_table = np.zeros((256, 256, 256), dtype=np.uint8)
        if color_path.endswith('.yaml'):
            with open(color_path, 'r') as stream:
                try:
                    color_values = yaml.safe_load(stream)
                except yaml.YAMLError as exc:
                    rospy.logerr(exc, logger_name="vision_pixellist_color_detector")

        # pickle-file is stored as '.pickle'
        elif color_path.endswith('.pickle'):
            try:
                with open(color_path, 'rb') as f:
                    color_values = pickle.load(f)
            except pickle.PickleError as exc:
                rospy.logerr(exc, logger_name="vision_pixellist_color_detector")

        # compatibility with colorpicker
        if 'color_values' in color_values.keys():
            color_values = color_values['color_values']['greenField']
        length = len(color_values['red'])
        if length == len(color_values['green']) and \
                length == len(color_values['blue']):
            # setting colors from yaml file to True in color lookup table
            for x in range(length):
                color_lookup_table[color_values['blue'][x], color_values['green'][x], color_values['red'][x]] = 1
        return color_lookup_table

    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns, if bgr pixel is in color lookup table

        :param np.array pixel: bgr-pixel
        :return bool: whether pixel is in color lookup table or not
        """
        return self._color_lookup_table[pixel[0], pixel[1], pixel[2]]

    def _mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Returns the color mask of the image
        (0 for not in color range and 255 for in color range)

        :param np.array image: input image
        :return np.array: masked image
        """
        return VisionExtensions.maskImg(image, self._color_lookup_table)


class DynamicPixelListColorDetector(PixelListColorDetector):
    """
    The :class:`.DynamicPixelListColorDetector`'s color lookup table is initially loaded from color-lookup-table-file defined in config
    and optionally adjustable to changing color conditions (dynamic color lookup table).
    """
    def __init__(self, config, package_path):
        # type:(dict, str) -> None
        """
        Initialization of DynamicPixelListColorDetector.

        :param dict config: dictionary of the vision node configuration parameters
        :param str package_path: path of package
        :return: None
        """
        self._static_mask = None

        # Initialization of parent PixelListColorDetector.
        super(DynamicPixelListColorDetector, self).__init__(config, package_path)

        # Annotate global variable. The global is needed due to threading issues
        global _dyn_color_lookup_table
        _dyn_color_lookup_table = np.copy(self._color_lookup_table)

        # Annotate global variable. The global is needed due to threading issues
        global _base_color_lookup_table
        _base_color_lookup_table = np.copy(self._color_lookup_table)

    def set_image(self, image):
        # type: (np.array) -> None
        """
        Refreshes class variables after receiving an image

        :param image: the current frame of the video feed
        :return: None
        """
        self._static_mask = None

        super(DynamicPixelListColorDetector, self).set_image(image)

    def get_static_mask_image(self, optional_image=None):
        # type: (np.array) -> np.array
        """
        Returns the color mask of the cached (or optional given) image based on the static color lookup table
        (0 for not in color range and 255 for in color range)

        :param np.array optional_image: Optional input image
        :return np.array: masked image
        """
        global _base_color_lookup_table

        if optional_image is not None:
            # Mask of optional image
            mask = self._mask_image(optional_image, _base_color_lookup_table)
        else:
            # Mask of default cached image
            mask = self._static_mask
            if mask is None:  # Check for cached static mask
                mask = self._static_mask = self._mask_image(self._image, _base_color_lookup_table)

        return mask

    def _mask_image(self, image, color_lookup_table=None):
        # type: (np.array) -> np.array
        """
        Returns the color mask of the image based on the dynamic color lookup table unless other is specified
        (0 for not in color range and 255 for in color range)

        :param np.array image: input image
        :param np.array color_lookup_table: Optional color lookup table
        :return np.array: masked image
        """
        if color_lookup_table is None:
            global _dyn_color_lookup_table
            color_lookup_table = _dyn_color_lookup_table

        return VisionExtensions.maskImg(image, color_lookup_table)

    def color_lookup_table_callback(self, msg):
        # type: (ColorLookupTable) -> None
        """
        This callback gets called inside the vision node, after subscriber received ColorLookupTableMessage from DynamicColorLookupTable-Node.

        :param ColorLookupTableMessage msg: ColorLookupTableMessage
        :return: None
        """
        self._decode_color_lookup_table(msg)

    def _decode_color_lookup_table(self, msg):
        # type: (ColorLookupTableMessage) -> None
        """
        Imports new color lookup table from ros msg. This is used to communicate with the DynamicColorLookupTable-Node.

        :param ColorLookupTableMessage msg: ColorLookupTableMessage
        :return: None
        """
        # Create temporary color lookup table
        # Use the base color lookup table as basis
        global _base_color_lookup_table
        color_lookup_table_temp = np.copy(_base_color_lookup_table)

        # Adds new colors to that color lookup table
        color_lookup_table_temp[
            msg.blue,
            msg.green,
            msg.red] = 1

        # Switches the reference to the new color lookup table
        global _dyn_color_lookup_table
        _dyn_color_lookup_table = color_lookup_table_temp
