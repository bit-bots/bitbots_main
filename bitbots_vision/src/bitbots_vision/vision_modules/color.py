import numpy as np
import VisionExtensions
from .debug import DebugPrinter
import yaml
import pickle
import time
import abc
import cv2
import multiprocessing
from collections import deque
# TODO Debug
import rospy
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from profilehooks import profile
from sets import Set


class ColorDetector:

    def __init__(self, debug_printer):
        self._debug_printer = debug_printer
        pass

    @abc.abstractmethod
    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns if bgr pixel is in color space

        :param bgr pixel:
        :return: whether pixel is in color space or not
        """

    @abc.abstractmethod
    def mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Creates a color mask
        (0 for not in color range and 255 for in color range)

        :param image: image to mask
        :return: masked image
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
        pic = np.zeros((1, 1, 3), np.uint8)
        pic[0][0] = pixel
        return cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)[0][0]


class HsvSpaceColorDetector(ColorDetector):

    def __init__(self, debug_printer, min_vals, max_vals):
        # type: (tuple, tuple, DebugPrinter) -> None
        """
        the hsv-space color detector

        :param min_vals: a tuple of the minimal accepted hsv-values
        :param max_vals: a tuple of the maximal accepted hsv-values
        :return: None
        """
        ColorDetector.__init__(self, debug_printer)
        self.min_vals = np.array(min_vals)
        self.max_vals = np.array(max_vals)

    def set_config(self, min_vals, max_vals):
        # type: (tuple[int, int], tuple[int, int]) -> None
        """
        updates the hsv-space configuration
        :param min_vals: a tuple of the minimal accepted hsv-values
        :param max_vals: a tuple of the maximal accepted hsv-values
        :return: None
        """
        self.min_vals = np.array(min_vals)
        self.max_vals = np.array(max_vals)

    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        pixel = self.pixel_bgr2hsv(pixel)
        # TODO: optimize comparisons
        return (self.max_vals[0] >= pixel[0] >= self.min_vals[0]) and \
               (self.max_vals[1] >= pixel[1] >= self.min_vals[1]) and \
               (self.max_vals[2] >= pixel[2] >= self.min_vals[2])

    def mask_image(self, image):
        # type: (np.array) -> np.array
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

    def init_color_space(self, color_path):
        # type: (str) -> None
        """
        Initializes color space from yaml or pickle.txt file

        :param color_path: path to file containing the accepted colors

        """
        color_space = np.zeros((256, 256, 256), dtype=np.uint8)
        if color_path.endswith('.yaml'):
            with open(color_path, 'r') as stream:
                try:
                    color_values = yaml.load(stream)
                except yaml.YAMLError as exc:
                    self._debug_printer.error(exc, 'PixelListColorDetector')
                    # Todo: what now??? Handle the error?
        # our pickle is stored as .txt
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


class StaticPixelListColorDetector(PixelListColorDetector):

    def __init__(self, debug_printer, color_path):
        ColorDetector.__init__(self, debug_printer)
        self.color_space = self.init_color_space(color_path)
        # TODO remove
        self.dc = DynamicPixelListColorDetector(debug_printer, color_path, 10, 0.6, 3)
        self.bridge = CvBridge()
        self.imagepublisher = rospy.Publisher("/mask_image_dyn", Image, queue_size=1)
        self.imagepublisher1 = rospy.Publisher("/mask_image", Image, queue_size=1)

    def match_pixel(self, pixel):
        # type: (tuple) -> bool
        """
        Returns if bgr pixel is in color space

        :param tuple pixel:
        :return bool: whether pixel is in color space or not
        """
        return self.color_space[pixel[0], pixel[1], pixel[2]]

    def mask_image(self, image):
        mask = self.dc.mask_image(image)
        # TODO remove
        self.imagepublisher.publish(self.bridge.cv2_to_imgmsg(mask, '8UC1'))
        """
        # type: (np.array) -> np.array
        Creates a color mask
        (0 for not in color range and 255 for in color range)
        :param np.array image: image to mask
        :return: np.array masked image
        """
        mask1 = VisionExtensions.maskImg(image, self.color_space)
        self.imagepublisher1.publish(self.bridge.cv2_to_imgmsg(mask1, '8UC1'))

        return mask

    def get_colorspace(self):
        # TODO docu
        return self.color_space

    #TODO remove
    def set_hor(self, hor):
        self.dc.set_horrizon_detector(hor)


class DynamicPixelListColorDetector(PixelListColorDetector):

    # TODO dynamic reconfigure kernel size
    # TODO docu
    def __init__(self, debug_printer, color_path, queue_max_size, threshold, kernel_size=3):
        """
        Colordetector is able to dynamically adapt colorspace to changing light- and field-conditions

        :param color_path: path of base colorspace
        :param queue_max_size: specifies size of the queue that hold lists of added colorvalues from the latest iterations
        :param threshold: amount of detected colorpixels in neighborhood of each pixel needed to be added to colorspace
            (in percentage between 0..1)
        :param kernel_size: kernel-size specifies the size of neighborhood in a square of pixels (ODD VALUE), DEFAULT=3
        """

        ColorDetector.__init__(self, debug_printer)

        self._base_color_space = self.init_color_space(color_path)

        self._dyn_color_space = np.copy(self._base_color_space)
        self._new_color_value_queue = deque(maxlen=queue_max_size)
        self._threshold = threshold
        self._kernel_size = kernel_size
        self._pointfinder = Pointfinder(debug_printer, self._threshold, self._kernel_size)
        self._heuristic = Heuristic(debug_printer)
        self.mask = None
        self.image = None

    def match_pixel(self, pixel):
        # type: (tuple) -> bool
        """
        Returns if bgr pixel is in color space

        :param tuple pixel:
        :return bool: whether pixel is in color space or not
        """
        # This is maybe not the colorspace from the current frame, because no image for an recalculation is given.
        return self._dyn_color_space[pixel[0], pixel[1], pixel[2]]

    def mask_image(self, image):
        # type: (np.array) -> np.array
        """
        Creates a color mask
        (0 for not in color range and 255 for in color range)

        :param np.array image: image to mask
        :return: np.array masked image
        """
        if np.array_equal(image, self.image):
            #TODO timeing mit horizon anpassen
            self.calc_dynamic_colorspace(image)
        else:
            self.image = image
            self.mask = self.mask_image_unwarpped(image)
        return self.mask

    def mask_image_unwarpped(self, image):
        mask = VisionExtensions.maskImg(image, self._dyn_color_space)
        return mask

    # TODO remove 
    @profile
    def calc_dynamic_colorspace(self, image):
        mask_image = self.mask_image_unwarpped(image)
        colorpixel_candidates_list = self._pointfinder.find_colorpixel_candidates(mask_image)
        colors = self.get_pixel_values(image, colorpixel_candidates_list)
        colors = np.array(self._heuristic.run(colors, image), dtype=np.int32)
        self._new_color_value_queue.append(colors)
        self.queue_to_colorspace()
        
    def queue_to_colorspace(self):
        self._dyn_color_space = np.copy(self._base_color_space)
        for new_color_value_list in self._new_color_value_queue:
            red = new_color_value_list[:, 0]
            green = new_color_value_list[:, 1]
            blue = new_color_value_list[:, 2]
            self._dyn_color_space[red, green, blue] = 1

    def get_pixel_values(self, img, pixellist):
        colors = img[pixellist[0], pixellist[1]]
        if colors.size > 0:
            unique_colors = np.unique(colors, axis=0)
        else:
            unique_colors = colors
        return unique_colors

    def set_horrizon_detector(self, horrizon_detector):
        self._heuristic.set_horrizon_detector(horrizon_detector)

class Pointfinder():
    def __init__(self, debug_printer, threshold, kernel_size=3):
        # type: (float, int) -> None
        """
        :param float threshold: necessary amount of true color in percentage (between 0..1)
        :param int kernel_size: defines size of convolution kernel, use odd number (default 3)
        """
        self._threshold = threshold
        self._kernel = np.ones((kernel_size, kernel_size))
        self._kernel[int(np.size(self._kernel, 0) / 2), int(np.size(self._kernel, 1) / 2)] = -self._kernel.size

    def find_colorpixel_candidates(self, masked_image):
        # type () -> np.array
        """
        Returns list of indices of pixel-candidates
        set masked image first

        :return: np.array: list of indices
        """
        normalized_image = np.divide(masked_image, 255, dtype=np.int16)
        sum_array = cv2.filter2D(normalized_image, -1, self._kernel, borderType=0)
        return np.array(np.where(sum_array > self._threshold * (self._kernel.size - 1)))


class Heuristic:

    def __init__(self, debug_printer, horrizon_detector=None):
        self._distribution_over_horizon = {}
        self._distribution_under_horizon = {}

        # Subprocess stuff
        self.cond = multiprocessing.Condition()
        self.manager = multiprocessing.Manager()
        # Shared memory variables
        self.process_is_waiting = multiprocessing.Value('b', True)
        self.shared_list = self.manager.list()
        # Setup Toolbox
        self.tools = HeuristicTools()
        self.debug_printer = debug_printer
        self.horrizon_detector = horrizon_detector

    def set_horrizon_detector(self, horrizon_detector):
        self.horrizon_detector = horrizon_detector

    def run(self, color_list, image, custom_mask=None):
        if self.horrizon_detector is None:
            print("Heuristic not used.")
            return color_list
        else:
            mask = self.horrizon_detector.get_mask()
            if mask is None:
                print("No Mask...")
                return np.array([])
        self.recalc(image, mask)
        return self.filter_colors(color_list)

    def filter_colors(self, color_list):
        color_list = self.tools.serialize(color_list)
        color_list_set = set(color_list)
        color_set_under = set(self._distribution_under_horizon)
        color_set_under = color_list_set.intersection(color_set_under)
        return self.tools.deserialize(np.array(list(color_set_under)))

    def recalc(self, image, mask):
        # Process-safe parameter exchange mask + image -> safe_color_freq
        if self.process_is_waiting.value:
            # Pulls the results from the other process.
            if len(self.shared_list) > 0:
                safe_allowedcolors = self.shared_list[0]
                self._distribution_under_horizon = safe_allowedcolors
                # ---- Process controll ----
            with self.process_is_waiting.get_lock():
                # Resets the waiting indicator.
                self.process_is_waiting.value = False
            with self.cond:
                # Allows the process to finish.
                self.cond.notify()
            # Restarts process with current image.
            HeuristicProcess(self.debug_printer, self.cond, self.process_is_waiting, self.shared_list, image, mask).start()


class HeuristicProcess(multiprocessing.Process):
    def __init__(self, debug_printer, cond, is_waiting, shared_list, image, mask, maxvalue=9999999):
        multiprocessing.Process.__init__(self)
        self.is_waiting = is_waiting
        self.cond = cond
        self.image = image
        self.mask = mask
        self.shared_list = shared_list
        # maximum number of output colors
        self.maxvalue = maxvalue
        # Setup Toolbox
        self.tools = HeuristicTools()

    def sort_by_values(self, array):
        return array[:, np.argsort(array[1])]

    def run(self):
        print("Working...")
        now = time.time()
        # Checks if Process has the Data
        if self.image is not None and self.mask is not None:
            # Limiting the output
            colors_over_horizon, colors_under_horizon = self.calc_freq()
            allowed_colors = set(colors_under_horizon) - set(colors_over_horizon)
            number_elements = len(allowed_colors)
            if self.maxvalue < number_elements:
                number_elements = self.maxvalue
            reduced_colors = random.sample(allowed_colors, number_elements)
            # Transmitting the output
            del self.shared_list[:]
            self.shared_list.append(reduced_colors)
        else:
            print("No image or mask given!!!")
        # Process waiting for it's death
        with self.cond:
            print("Waiting")
            with self.is_waiting.get_lock():
                self.is_waiting.value = True
            print(time.time() - now)
            self.cond.wait(5)
        print("I got killed")

    def calc_freq(self):
        img = self.image
        mask = self.mask
        # Calls a funktion to calculate the number of occurencies of all colors in the image
        # returns array(over_horrizon(unique, count), under_horrizon(unique, count))
        return (self.get_freq_colors(cv2.bitwise_and(img, img, mask=255 - mask)),
                self.get_freq_colors(cv2.bitwise_and(img, img, mask=mask)))

    def get_freq_colors(self, img):
        # Serializes image
        serialized_img = self.tools.serialize(np.reshape(img, (1, int(img.size / 3), 3))[0])
        # Calculates the number of occurencies of all colors in the image
        unique = np.unique(serialized_img, axis=0)
        return unique


class HeuristicTools():
    '''
    Small Toolbox for functions used in both the Heuristic and the HeuristicProcess class.
    '''

    def __init__(self):
        pass

    def serialize(self, input_matrix):
        return np.array(np.multiply(input_matrix[:, 0], 256 ** 2) \
                        + np.multiply(input_matrix[:, 1], 256) \
                        + input_matrix[:, 2], dtype=np.int32)

    def deserialize(self, input_matrix):
        new_matrix = np.zeros((input_matrix.size, 3))
        new_matrix[:, 0] = np.array(input_matrix // 256 ** 2, dtype=np.uint8)
        new_matrix[:, 1] = np.array((input_matrix - (new_matrix[:, 0] * 256 ** 2)) / 256, dtype=np.uint8)
        new_matrix[:, 2] = np.array((input_matrix - (new_matrix[:, 0] * 256 ** 2) - (new_matrix[:, 1] * 256)),
                                    dtype=np.uint8)
        return new_matrix