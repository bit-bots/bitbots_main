#! /usr/bin/env python2
from cv2 import KeyPoint

# TODO: docs


class KeyPointConverter():

    def __init__(self):
        pass

    def key_point2values(self, kp):
        pt = kp.pt
        pt_x = pt[0]
        pt_y = pt[1]
        size = kp.size
        angle = kp.angle
        response = kp.response
        octave = kp.octave
        class_id = kp.class_id
        return (pt_x, pt_y, size, angle, response, octave, class_id)

    def values2key_points(self, pt_x, pt_y, size, angle, response, octave, class_id):
        return KeyPoint(pt_x, pt_y, size, angle, response, octave, class_id)
