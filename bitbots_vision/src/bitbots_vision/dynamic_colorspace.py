#! /usr/bin/env python2

################################################
####### TODO: dynamic reconfigure, docu ########
################################################

import rospy
import rospkg

class DynamicColorspace:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_dynamic_colorspace')

        rospy.init_node('bitbots_dynmaic_colorspace')
        rospy.loginfo('Initializing dynmaic colorspace...')

        self.config = {}

        rospy.spin()


class DynamicPixelListColorDetector(PixelListColorDetector):
    # Default: DynamicPixelListColorDetector(debug_printer, color_path, 10, 0.6, 3)
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

        # TODO remove
        self.imagepublisher = rospy.Publisher("/mask_image_dyn", Image, queue_size=1)

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

        # TODO remove
        self.imagepublisher.publish(self.bridge.cv2_to_imgmsg(mask, '8UC1'))
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
        self.debug_printer = debug_printer
        self.horrizon_detector = horrizon_detector

    def set_horrizon_detector(self, horrizon_detector):
        self.horrizon_detector = horrizon_detector

    def run(self, color_list, image, custom_mask=None):
        if self.horrizon_detector is None:
            print("Heuristic not used. Continue with static colorspace.")
            return np.array([])
        else:
            mask = self.horrizon_detector.get_mask()
            if mask is None:
                print("No Mask...")
                return np.array([])
        return self.filter_colors(color_list, image, mask)

    def filter_colors(self, color_list, image, mask):
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
