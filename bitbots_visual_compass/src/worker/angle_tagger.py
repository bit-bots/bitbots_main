from __future__ import absolute_import

import numpy as np
import math
import cv2

class AngleTagger:

    def __init__(self, rotation_matrix):
        #TODO
        #Logitech
        """
        rotation_matrix = np.array([[475.225100, 0.000000, 322.862838],
                        [0.000000, 478.700781, 174.340100],
                        [0.000000, 0.000000, 1.000000]])"""
        #Basler
        rotation_matrix = np.array([[1058.439188, 0.000000, 310.612349], 
                                    [0.000000, 1092.738996, 229.659998], 
                                    [0.000000, 0.000000, 1.000000]])
        self.inv_rot_mat = np.linalg.inv(rotation_matrix)

    def tag_keypoints(self, offset, keypoints):
        return [self.tag_one_keypoint(offset, keypoints[i]) for i in range(len(keypoints))]

    def tag_one_keypoint(self, offset, keypoint):
        vector = np.array([keypoint.pt[0], keypoint.pt[1], 1.])
        x = self.inv_rot_mat.dot(vector)[0]
        angle = (math.atan(x) + offset) % (math.pi * 2)
        return cv2.KeyPoint(.0, .0, offset, angle)
