#! /usr/bin/env python2

import rospy
import rospkg
import cv2
import time
import numpy as np
from bitbots_vision.vision_modules.debug import DebugPrinter
from cv_bridge import CvBridge
from bitbots_vision.vision_modules import horizon, color
from collections import deque
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from dynamic_colorspace.cfg import dynamic_colorspaceConfig
from bitbots_msgs.msg import Colorspace


class DynamicColorspace:
    def __init__(self):
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path('bitbots_vision')

        rospy.init_node('bitbots_dynamic_colorspace')
        rospy.loginfo('Initializing dynmaic colorspace...')

        self._config = {}

        Server(dynamic_colorspaceConfig, self._dynamic_reconfigure_callback)
        
        self._bridge = CvBridge()

        self._debug_printer = None

        # TODO dyn reconf
        self._queue_max_size = 10
        self._threshold = 0.6
        self._kernel_size = 3

        self._color_detector = color.PixelListColorDetector(
            self._debug_printer,
            self._package_path +
            self._config['field_color_detector_path'])

        self._horizon_detector = horizon.HorizonDetector(
            self._color_detector,
            self._config,
            self._debug_printer)

        self._color_value_queue = deque(maxlen=self._queue_max_size)

        self._pointfinder = Pointfinder(self._debug_printer, self._threshold, self._kernel_size)
        self._heuristic = Heuristic(self._debug_printer)

        self._image_sub = rospy.Subscriber('image_raw',
                                          Image,
                                          self._image_callback,
                                          queue_size=1,
                                          tcp_nodelay=True,
                                          buff_size=2**20)

        self._colorspace_publisher = rospy.Publisher('colorspace',
                                                    Colorspace,
                                                    queue_size=1)
        rospy.spin()

    def calc_dynamic_colorspace(self, image):
        # Masks new image with current colorspace
        mask_image = self._color_detector.mask_image(image)
        # Calls the horizon detector
        self._horizon_detector.set_image(image)
        self._horizon_detector.compute_horizon_points()
        mask = self._horizon_detector.get_mask()
        # Searches for new candidate pixels in the color mask
        colorpixel_candidates_list = self._pointfinder.find_colorpixel_candidates(mask_image)
        # Gets unique color vales from the candidate pixels.
        colors = self.get_pixel_values(image, colorpixel_candidates_list)
        # Filters the colors using the heuristic. 
        colors = np.array(self._heuristic.run(colors, image, mask), dtype=np.int32)
        # Adds new (sub-)colorspace to the queue
        self._color_value_queue.append(colors)
        
    def queue_to_colorspace(self):
        # Inizializes an empty colorspace
        colorspace = np.array([]).reshape(0,3)
        # Stacks every colorspace in the queue
        for new_color_value_list in self._color_value_queue:
            colorspace = np.append(colorspace, new_color_value_list[:,:], axis=0)
        # Returns an colorspace, which contains all colors from the queue
        return colorspace

    def get_pixel_values(self, img, pixellist):
        '''
        Returns unique colors for an given image and a list of pixels.
        '''
        colors = img[pixellist[0], pixellist[1]]
        if colors.size > 0:
            unique_colors = np.unique(colors, axis=0)
        else:
            unique_colors = colors
        return unique_colors

    def publish(self, image_msg):
        '''
        Publishes the current colorspace via ros msg.
        '''
        colorspace = self.queue_to_colorspace()
        colorspace_msg = Colorspace()  # Todo: add lines
        colorspace_msg.header.frame_id = image_msg.header.frame_id
        colorspace_msg.header.stamp = image_msg.header.stamp
        colorspace_msg.blue  = colorspace[:,0].tolist()
        colorspace_msg.green = colorspace[:,1].tolist()
        colorspace_msg.red   = colorspace[:,2].tolist()
        self._colorspace_publisher.publish(colorspace_msg)

    def _image_callback(self, img):
        # Sometimes the queue gets to large, even when the size is limeted to 1. 
        # Thats why we drop old images manualy.
        # Drops old images
        delta = rospy.get_rostime() - img.header.stamp 
        if delta.to_sec() > 0.1:
            return
        # Converting the ROS image message to CV2-image
        image = self._bridge.imgmsg_to_cv2(img, 'bgr8')
        # Calculates the new colorspace
        self.calc_dynamic_colorspace(image)
        # Publishes the colorspace
        self.publish(img)

    def _dynamic_reconfigure_callback(self, config, level):
        self._config = config

        # TODO: debug-image on/off

        if (self._queue_max_size != self._config['queue_max_size']):
            self._queue_max_size = self._config['queue_max_size']
            self._color_value_queue = deque(maxlen=self._queue_max_size)

        # Pointfinder
        self._threshold = self._config['threshold']
        self._kernel_size = self._config['kernel_size']
        self._pointfinder.set_params(self._threshold, self._kernel_size)
        return config
        


class Pointfinder():
    def __init__(self, debug_printer, threshold, kernel_size=3):
        # type: (float, int) -> None
        """
        :param float threshold: necessary amount of true color in percentage (between 0..1)
        :param int kernel_size: defines size of convolution kernel, use odd number (default 3)
        """
        self._threshold = threshold
        # Defines kernel
        self._kernel = np.ones((kernel_size, kernel_size))
        self._kernel[int(np.size(self._kernel, 0) / 2), int(np.size(self._kernel, 1) / 2)] = -self._kernel.size

    def set_params(self, threshold, kernel_size=3):
        """
        TODO doku
        dynamic reconfigure -> update params 
        """
        self._threshold = threshold
        self._kernel = np.ones((kernel_size, kernel_size))
        self._kernel[int(np.size(self._kernel, 0) / 2), int(np.size(self._kernel, 1) / 2)] = -kernel.size

    def find_colorpixel_candidates(self, masked_image):
        # type () -> np.array
        """
        Returns list of indices of pixel-candidates
        set masked image first

        :return: np.array: list of indices
        """
        # Normalizes the image to values between 1 und 0
        normalized_image = np.divide(masked_image, 255, dtype=np.int16)
        # Calculates the number of neighbours or each pixel
        sum_array = cv2.filter2D(normalized_image, -1, self._kernel, borderType=0)
        # Returns all pixels with an higher neighbour / non neighbours ratio than the threshold
        return np.array(np.where(sum_array > self._threshold * (self._kernel.size - 1)))

class Heuristic:

    def __init__(self, debug_printer, horrizon_detector=None):
        self._debug_printer = debug_printer
        self._horrizon_detector = horrizon_detector

    def run(self, color_list, image, mask):
        # Simplyfies the handling by merging the 3 color channels
        color_list = self.serialize(color_list)
        color_set = set(color_list)
        # Generates whitelist
        whitelist = self.recalc(image, mask)
        # Takes only whitelisted values 
        color_set = color_set.intersection(whitelist)
        # Restrucktures the color channels
        return self.deserialize(np.array(list(color_set)))

    def recalc(self, image, mask):
        # Genereates whitelist
        colors_over_horizon, colors_under_horizon = self.calc_freq(image, mask)
        return set(colors_under_horizon) - set(colors_over_horizon)

    def calc_freq(self, image, mask):
        # Calls a funktion to calculate the number of occurencies of all colors in the image
        # returns array(over_horrizon(unique, count), under_horrizon(unique, count))
        return (self.get_freq_colors(cv2.bitwise_and(image, image, mask=255 - mask)),
                self.get_freq_colors(cv2.bitwise_and(image, image, mask=mask)))

    def get_freq_colors(self, image):
        # Simplyfies the handling by merging the 3 color channels
        serialized_img = self.serialize(np.reshape(image, (1, int(image.size / 3), 3))[0])
        # Returns unique colors in the image
        return np.unique(serialized_img, axis=0)

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


if __name__ == '__main__':
    DynamicColorspace()