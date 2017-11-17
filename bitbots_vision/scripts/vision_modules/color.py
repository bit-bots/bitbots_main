import numpy as np
import yaml


class ColorDetector:

    def __init__(self, color_path):
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

    def match_pixel(self, pixel) -> bool:
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
