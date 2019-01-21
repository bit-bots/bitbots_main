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
from bitbots_vision.cfg import VisionConfig
from dynamic_reconfigure.server import Server
from bitbots_msgs.msg import Colorspace
from std_msgs.msg import UInt8
from rospy.numpy_msg import numpy_msg

# TODO remove
from profilehooks import profile 


class DynamicColorspace:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_vision')

        rospy.init_node('bitbots_dynamic_colorspace')
        rospy.loginfo('Initializing dynmaic colorspace...')

        self.config = {}

        Server(VisionConfig, self._dynamic_reconfigure_callback)
        
        print("Loaded")

        self.bridge = CvBridge()

        self.debug_printer = None

        # TODO dyn reconf
        queue_max_size = 10
        self._threshold = 0.6
        self._kernel_size = 3

        self.color_detector = color.PixelListColorDetector(
            self.debug_printer,
            self.package_path +
            self.config['field_color_detector_path'])

        self.horizon_detector = horizon.HorizonDetector(
            self.color_detector,
            self.config,
            self.debug_printer)

        self._new_color_value_queue = deque(maxlen=queue_max_size)

        self._pointfinder = Pointfinder(self.debug_printer, self._threshold, self._kernel_size)
        self._heuristic = Heuristic(self.debug_printer)

        self.image_sub = rospy.Subscriber('image_raw',
                                          Image,
                                          self._image_callback,
                                          queue_size=1,
                                          tcp_nodelay=True,
                                          buff_size=2**20)

        self.colorspace_publisher = rospy.Publisher('colorspace',
                                                    Colorspace,
                                                    queue_size=1)

        print("Ready")
        rospy.spin()

    # TODO remove 
    @profile
    def calc_dynamic_colorspace(self, image):
        mask_image = self.color_detector.mask_image(image)
        self.horizon_detector.set_image(image)
        self.horizon_detector.compute_horizon_points()
        mask = self.horizon_detector.get_mask()
        colorpixel_candidates_list = self._pointfinder.find_colorpixel_candidates(mask_image)
        colors = self.get_pixel_values(image, colorpixel_candidates_list)
        colors = np.array(self._heuristic.run(colors, image, mask), dtype=np.int32)
        self._new_color_value_queue.append(colors)
        
    def queue_to_colorspace(self):
        colorspace = np.array([]).reshape(0,3)
        for new_color_value_list in self._new_color_value_queue:
            colorspace = np.append(colorspace, new_color_value_list[:,:], axis=0)
        return colorspace

    def get_pixel_values(self, img, pixellist):
        colors = img[pixellist[0], pixellist[1]]
        if colors.size > 0:
            unique_colors = np.unique(colors, axis=0)
        else:
            unique_colors = colors
        return unique_colors

    # TODO remove 
    @profile
    def publish(self, image_msg):
        colorspace = np.array(self.queue_to_colorspace(), dtype=np.uint8)
        colorspace_msg = Colorspace()  # Todo: add lines
        colorspace_msg.header.frame_id = image_msg.header.frame_id
        colorspace_msg.header.stamp = image_msg.header.stamp
        colorspace_msg.blue  = colorspace[:,0].tolist()
        colorspace_msg.green = colorspace[:,1].tolist()
        colorspace_msg.red   = colorspace[:,2].tolist()
        self.colorspace_publisher.publish(colorspace_msg)

    def _image_callback(self, img):
        delta = rospy.get_rostime() - img.header.stamp 
        if delta.to_sec() > 0.1:
            return
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        self.calc_dynamic_colorspace(image)
        self.publish(img)

    def _dynamic_reconfigure_callback(self, config, level):
        # TODO Stuff
        self.config = config
        return config
        


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
        self.debug_printer = debug_printer
        self.horrizon_detector = horrizon_detector

    def set_horrizon_detector(self, horrizon_detector):
        self.horrizon_detector = horrizon_detector

    def run(self, color_list, image, mask):
        color_list = self.serialize(color_list)
        color_set = set(color_list)
        colors_set_under_horizon = set(self.recalc(image, mask))
        color_set = color_set.intersection(colors_set_under_horizon)
        return self.deserialize(np.array(list(color_set)))

    def recalc(self, image, mask):
        colors_over_horizon, colors_under_horizon = self.calc_freq(image, mask)
        return set(colors_under_horizon) - set(colors_over_horizon)

    def calc_freq(self, image, mask):
        # Calls a funktion to calculate the number of occurencies of all colors in the image
        # returns array(over_horrizon(unique, count), under_horrizon(unique, count))
        return (self.get_freq_colors(cv2.bitwise_and(image, image, mask=255 - mask)),
                self.get_freq_colors(cv2.bitwise_and(image, image, mask=mask)))

    def get_freq_colors(self, image):
        # Serializes image
        serialized_img = self.serialize(np.reshape(image, (1, int(image.size / 3), 3))[0])
        # Calculates the number of occurencies of all colors in the image
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