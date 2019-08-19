#! /usr/bin/env python2
from cv2 import KeyPoint

# TODO: docs


class KeyPointConverter():
    """
    Abuses the cv2 keypoint class to store additional data.
    """
    def __init__(self):
        pass

    def key_point2values(self, kp):
        """
        :param kp: CV2 Keypoint instance
        :return tuple: Tuple that contains (keypoint x position, keypoint y position, keypoint size, keypoint angle, ...
        """
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
        """
        A better constructor for the CV2 Keypoint class
        """
        return KeyPoint(pt_x, pt_y, size, angle, response, octave, class_id)
