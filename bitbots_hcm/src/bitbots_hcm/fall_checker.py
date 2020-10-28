# -*- coding: utf8 -*-
import array
import math
import numpy
from sensor_msgs.msg import Imu
import rospy

# todo hip pitch offset
from sklearn.base import BaseEstimator
from sklearn.metrics import accuracy_score


class FallChecker(BaseEstimator):

    def __init__(self, thresh_gyro_front=rospy.get_param("hcm/threshold_gyro_y_front"),
                 thresh_gyro_side=rospy.get_param("hcm/threshold_gyro_x_side"),
                 thresh_orient_front=math.radians(rospy.get_param("hcm/falling_threshold_orientation_front_back")),
                 tresh_orient_side=math.radians(rospy.get_param("hcm/falling_threshold_orientation_left_right"))):

        self.thresh_gyro_front = thresh_gyro_front
        self.thresh_gyro_side = thresh_gyro_side
        self.thresh_orient_front = thresh_orient_front
        self.tresh_orient_side = tresh_orient_side

        self.STABLE = 0
        self.FRONT = 1
        self.BACK = 2
        self.LEFT = 3
        self.RIGHT = 4

    def check_falling(self, not_much_smoothed_gyro, euler):
        """Checks if the robot is currently falling and in which direction. """
        # Checks if robot is still
        bools = [abs(n) < 0.1 for n in not_much_smoothed_gyro]
        if bools[0] and bools[1] and bools[2]:
            return self.STABLE

        # setting the fall quantification function
        x_fall_quantification = self.calc_fall_quantification(
            self.tresh_orient_side,
            self.thresh_gyro_front,
            euler[0],
            not_much_smoothed_gyro[0])

        y_fall_quantification = self.calc_fall_quantification(
            self.thresh_orient_front,
            self.thresh_gyro_side,
            euler[1],
            not_much_smoothed_gyro[1])

        if x_fall_quantification + y_fall_quantification == 0:
            return self.STABLE

        # compare quantification functions
        if y_fall_quantification > x_fall_quantification:
            # detect the falling direction
            if not_much_smoothed_gyro[1] < 0:
                return self.BACK
            # detect the falling direction
            else:
                return self.FRONT
        else:
            # detect the falling direction
            if not_much_smoothed_gyro[0] < 0:
                return self.LEFT
            # detect the falling direction
            else:
                return self.RIGHT

    def calc_fall_quantification(self, falling_threshold_orientation, falling_threshold_gyro, current_axis_euler,
                                 current_axis__gyro):
        # check if you are moving forward or away from the perpendicular position, by comparing the signs.
        if numpy.sign(current_axis_euler) == numpy.sign(current_axis__gyro):
            # calculatiung the orentation skalar for the threshold
            skalar = max((falling_threshold_orientation - abs(current_axis_euler)) / falling_threshold_orientation, 0)
            # checking if the rotation velocity is lower than the the threshold
            if falling_threshold_gyro * skalar < abs(current_axis__gyro):
                # returning the fall quantification function
                return abs(current_axis__gyro) * (1 - skalar)
        return 0

    def fit(self, x, y):
        # we have to do nothing, as we are not actually fitting any model
        pass

    def score(self, X, y, sample_weight=None):
        return accuracy_score(y, self.predict(X), sample_weight=sample_weight)

    def predict(self, x):
        # only take gyro and orientation from data
        y = []
        for entry in x:
            prediction = self.check_falling(entry[3:6], entry[6:9])
            y.append(prediction)
        return y
