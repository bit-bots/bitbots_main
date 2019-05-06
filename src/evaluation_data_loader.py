#!/usr/bin/env python2
import cv2
import math
import yaml
import os

class DataLoader(object):

    def __init__(self, data_path, dimensions, angle_steps):
        self.data_path = data_path
        index_path = os.path.join(self.data_path, "index.yaml")
        with open(index_path, 'r') as stream:
            self.index = yaml.load(stream)
        self.meta_data = self._sortImages(self.index, dimensions, angle_steps)
        
    def _sortImages(self, index, dimensions, angle_steps):
        sortedMetaData = dict()

        for r in range(dimensions[0]):
            sortedMetaData[r] = dict()
            for c in range(dimensions[1]):
                sortedMetaData[r][c] = dict()

        for image in index:
            angle_key = int(round((image['angle']/(2*math.pi))*16))
            sortedMetaData[image['row']][image['checkpoint']][angle_key] = {'angle': image['angle'], 
                                                                            'path': str(image['path'])}
        
        return sortedMetaData

    def getMetaDataSetForCheckpoint(self, row, checkpoint):
        return self.meta_data[row][checkpoint]

    def getMetaDataSetForImage(self, row, checkpoint, angle):
        return self.meta_data[row][checkpoint][int(round((angle/(2*math.pi))*16))]

    def getImage(self, row, checkpoint, angle):
        meta_data = self.getMetaDataSetForImage(row, checkpoint, angle)
        image_path = os.path.join(self.data_path, meta_data['path'])
        return cv2.imread(image_path)

