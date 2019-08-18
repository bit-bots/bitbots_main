#!/usr/bin/env python2
import cv2
import math
import yaml
import os


class DataLoader(object):
    """
    Queries data from the field data image set.
    """
    def __init__(self, data_path, dimensions, angle_steps):
        self.data_path = data_path
        index_path = os.path.join(self.data_path, "index.yaml")
        with open(index_path, 'r') as stream:
            self.index = yaml.load(stream)
        self.meta_data = self._sort_images(self.index, dimensions, angle_steps)

    def _sort_images(self, index, dimensions, angle_steps):
        sorted_meta_data = dict()

        for r in range(dimensions[0]):
            sorted_meta_data[r] = dict()
            for c in range(dimensions[1]):
                sorted_meta_data[r][c] = dict()

        for image in index:
            angle_key = int(round((image['angle']/(2*math.pi))*16))
            sorted_meta_data[image['row']][image['checkpoint']][angle_key] = {'angle': image['angle'],
                                                                            'path': str(image['path'])}

        return sorted_meta_data

    def _get_meta_data_set_for_checkpoint(self, row, checkpoint):
        return self.meta_data[row][checkpoint]

    def _get_meta_data_set_for_image(self, row, checkpoint, angle):
        angle_key = int(round((angle / (2 * math.pi)) * 16)) % 16
        return self.meta_data[row][checkpoint][angle_key]

    def get_image(self, row, checkpoint, angle):
        meta_data = self._get_meta_data_set_for_image(row, checkpoint, angle)
        image_path = os.path.join(self.data_path, meta_data['path'])
        return cv2.imread(image_path)

