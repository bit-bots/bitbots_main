import numpy as np
import yaml
import abc
import cv2


class ColorDetector:

    def __init__(self):
        pass

    @abc.abstractmethod
    def match_pixel(self, pixel):
        """
        Returns if bgr pixel is in color space

        :param bgr pixel:
        :return whether pixel is in color space or not
        """

    @abc.abstractmethod
    def mask_image(self, image):
        """
        Creates a color mask
        (0 for not in color range and 255 for in color range)
        :param image: image to mask
        :return: masked image
        """


class PixelListColorDetector(ColorDetector):
    def __init__(self, color_path):
        ColorDetector.__init__(self)
        self.color_space = np.full((256, 256, 256), False, dtype=bool)
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
        length = len(color_values['red'])
        if length == len(color_values['green']) and \
                        length == len(color_values['blue']):
            # setting colors from yaml file to True in color space
            for x in range(length):
                self.color_space[color_values['blue'][x],
                                 color_values['green'][x],
                                 color_values['red'][x]] = True

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
        imgshape = image.shape
        mask = np.zeros((imgshape[0], imgshape[1]))
        for row in range(imgshape[0]):
            for col in range(imgshape[1]):
                if self.match_pixel(image[row, col]):
                    mask[row, col] = 255
        return mask


class HsvSpaceColorDetector(ColorDetector):

    def __init__(self, min_vals, max_vals):
        ColorDetector.__init__(self)
        self.min_vals = min_vals
        self.max_vals = max_vals

    def match_pixel(self, pixel):
        pixel = self.pixel_bgr2hsv(pixel)
        return (pixel[0] <= self.max_vals[0]
                or pixel[0] >= self.min_vals[0]) and \
               (pixel[1] <= self.max_vals[1]
                or pixel[1] >= self.min_vals[1]) and \
               (pixel[2] <= self.max_vals[2]
                or pixel[3] >= self.min_vals[3])

    def mask_image(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(image, self.min_vals, self.max_vals)

    def pixel_bgr2hsv(self, pixel):
        pic = np.zeros((1, 1, 3), np.uint8)
        pic[0][0] = pixel
        return cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)[0][0]

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

