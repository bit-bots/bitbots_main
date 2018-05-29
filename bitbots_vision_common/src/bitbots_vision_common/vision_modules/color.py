import numpy as np
import VisionExtensions
import yaml
import abc
import cv2


class ColorDetector:

    def __init__(self):
        pass

    @abc.abstractmethod
    def match_pixel(self, pixel):
        # type: (np.array) -> bool
        """
        Returns if bgr pixel is in color space

        :param bgr pixel:
        :return whether pixel is in color space or not
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

    def match_adjacent(self, image, point, offset=5, threshold=200):
        # type: (np.array, tuple[int, int], int, float) -> bool
        """
        Returns if an area is in color space

        :param image: the full image
        :param point: a x-, y-tuple defining coordinates in the image
        :param offset: the number of pixels to check in the surounding of the
        point (like a radius but for a square)
        :param threshold: the mean needed to accept the area to match (0-255)
        :return whether area is in color space or not
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

        :param area: the image area to check
        :param threshold: the mean needed to accept the area to match (0-255)
        :return whether area is in color space or not
        """
        return np.mean(self.mask_image(area)) > threshold



    @staticmethod
    def pixel_bgr2hsv(pixel):
        # type: (np.array) -> np.array
        pic = np.zeros((1, 1, 3), np.uint8)
        pic[0][0] = pixel
        return cv2
class PixelListColorDetector(ColorDetector):
    def __init__(self, color_path):
        ColorDetector.__init__(self)
        self.color_space = np.zeros((256, 256, 256), dtype=np.uint8)
        self.init_color_space(color_path)

    def init_color_space(self, color_path):
        """
        Initializes color space from yaml file
        :param color_path: path to yaml file containing the accepted colors

        """
        with open(color_path, 'r') as stream:
            try:
                color_values = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                # Todo: what now??? Handle the error?
        # compatibility with colorpicker
        if 'color_values' in color_values.keys():
            color_values = color_values['color_values']['greenField']
        length = len(color_values['red'])
        if length == len(color_values['green']) and \
                        length == len(color_values['blue']):
            # setting colors from yaml file to True in color space
            for x in range(length):
                self.color_space[color_values['blue'][x],
                                 color_values['green'][x],
                                 color_values['red'][x]] = 1

    def match_pixel(self, pixel):
        """
        Returns if bgr pixel is in color space

        :param bgr pixel:
        :return whether pixel is in color space or not
        """
        return self.color_space[pixel[0], pixel[1], pixel[2]]

    def mask_image(self, image):
        """
        Creates a color mask
        (0 for not in color range and 255 for in color range)
        :param image: image to mask
        :return: masked image
        """
        mask = VisionExtensions.maskImg(image, self.color_space)
        return mask


class HsvSpaceColorDetector(ColorDetector):

    def __init__(self, min_vals, max_vals):
        ColorDetector.__init__(self)
        self.min_vals = np.array(min_vals)
        self.max_vals = np.array(max_vals)

    def set_config(self, min_vals, max_vals):
        self.min_vals = np.array(min_vals)
        self.max_vals = np.array(max_vals)

    def match_pixel(self, pixel):
        pixel = self.pixel_bgr2hsv(pixel)
        # TODO: optimize comparisons
        return (self.max_vals[0] >= pixel[0] >= self.min_vals[0]) and \
               (self.max_vals[1] >= pixel[1] >= self.min_vals[1]) and \
               (self.max_vals[2] >= pixel[2] >= self.min_vals[2])

    def mask_image(self, image):
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

