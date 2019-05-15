from __future__ import absolute_import

import bisect
import colorsys
import math
import cv2
import os
import yaml
import numpy as np
import time
from worker.matcher import Matcher
# from .worker.interface import VisualCompass as VisualCompassInterface
# from .worker.debug import Debug
from evaluation_data_loader import DataLoader


def show_img(image):
    cv2.imshow("Record", image)
    k = cv2.waitKey(0)

if __name__ == "__main__":
    dimensions = (10, 7)
    angle_steps = 16

    dirname = os.path.dirname(__file__)
    relative_config_path = "../config/config.yaml"
    config_path = os.path.join(dirname, relative_config_path)

    with open(config_path, 'r') as stream:
        config = yaml.load(stream)

    relative_data_path = config['evaluation_data']
    data_path = os.path.join(dirname, relative_data_path)

    dl = DataLoader(data_path, dimensions, angle_steps)

    image = dl.getImage(0,0,0)
    matcher = Matcher(config)
    kp = matcher.get_keypoints(image)[0]

    # image = cv2.drawKeypoints(image, kp, None, color=0x131313, flags=0)

    font = cv2.FONT_HERSHEY_SIMPLEX
    bottom_left_corner_of_text = (10, 35)
    font_scale = .5
    font_color = (255, 255, 255)
    line_type = 1




    print (kp[0].pt)



    promatrix = np.array([[470.998474, 0.000000, 327.080661, 0.000000],
                       [0.000000, 479.502167, 174.650992, 0.000000],
                       [0.000000, 0.000000, 1.000000,  0.000000],
                       [0.000000, 0.000000, 0.000000,  1.000000]])
    matrix  = np.array([[475.225100, 0.000000, 322.862838],
                        [0.000000, 478.700781, 174.340100],
                        [0.000000, 0.000000, 1.000000]])
    vector = np.array([52,52,1])
    proinverse = np.linalg.inv(matrix)

    def calc(pt):
        vector = np.array([pt[0], pt[1], 1]);
        return inverse.dot(vector)
    inverse = np.linalg.inv(matrix)
    print (inverse)
    #print (vector.dot(inverse))
    #print (inverse.dot(vector))
    print inverse.dot(vector)

    def f(x):
        #return int(x[0])
        return int(math.atan(x[0]) / (2*math.pi) * 360 )

    map(lambda x:
    cv2.putText(image, "{}".format(f(calc(x.pt))),
                (int(x.pt[0]), int(x.pt[1])),
                font,
                font_scale,
                font_color,
                line_type), kp )
    show_img(image)