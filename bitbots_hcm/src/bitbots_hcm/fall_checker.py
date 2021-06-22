# -*- coding: utf8 -*-
import math
import numpy
from functools import reduce

from sensor_msgs.msg import Imu
import rospy

from sklearn.base import BaseEstimator
from sklearn.metrics import accuracy_score


class FallChecker(BaseEstimator):

    def __init__(self, thresh_gyro_pitch=None,
                 thresh_gyro_roll=None,
                 thresh_orient_pitch=None,
                 thresh_orient_roll=None,
                 smoothing=None):
        self.thresh_gyro_pitch = rospy.get_param("hcm/falling_thresh_gyro_pitch") \
            if thresh_gyro_pitch is None else thresh_gyro_pitch
        self.thresh_gyro_roll = rospy.get_param("hcm/falling_thresh_gyro_roll") \
            if thresh_gyro_roll is None else thresh_gyro_roll
        self.thresh_orient_pitch = math.radians(rospy.get_param("hcm/falling_thresh_orient_pitch")) \
            if thresh_orient_pitch is None else thresh_orient_pitch
        self.thresh_orient_roll = math.radians(rospy.get_param("hcm/falling_thresh_orient_roll")) \
            if thresh_orient_roll is None else thresh_orient_roll

        self.smoothing = rospy.get_param("hcm/smooth_threshold") if smoothing is None else smoothing
        self.smoothing_list = []
        self.counter = 0
        self.last_result = 0

        self.STABLE = 0
        self.FRONT = 1
        self.BACK = 2
        self.LEFT = 3
        self.RIGHT = 4

    def update_reconfigurable_values(self, config, level):
        # Dynamic Reconfigure
        self.thresh_gyro_pitch = config["falling_thresh_gyro_pitch"]
        self.thresh_gyro_roll = config["falling_thresh_gyro_roll"]
        self.thresh_orient_pitch = math.radians(config["falling_thresh_orient_pitch"])
        self.thresh_orient_roll = math.radians(config["falling_thresh_orient_roll"])
        return config

    def check_falling(self, not_much_smoothed_gyro, quaternion):
        """Checks if the robot is currently falling and in which direction. """
        # Convert quaternion to fused angles
        fused_roll, fused_pitch, _ = self.fused_from_quat(quaternion)

        # setting the fall quantification function
        roll_fall_quantification = self.calc_fall_quantification(
            self.thresh_orient_roll,
            self.thresh_gyro_roll,
            fused_roll,
            not_much_smoothed_gyro[0])

        pitch_fall_quantification = self.calc_fall_quantification(
            self.thresh_orient_pitch,
            self.thresh_gyro_pitch,
            fused_pitch,
            not_much_smoothed_gyro[1])

        if roll_fall_quantification + pitch_fall_quantification == 0:
            result = self.STABLE
        else:
            # compare quantification functions
            if pitch_fall_quantification > roll_fall_quantification:
                # detect the falling direction
                if fused_pitch < 0:
                    result = self.BACK
                # detect the falling direction
                else:
                    result = self.FRONT
            else:
                # detect the falling direction
                if fused_roll < 0:
                    result = self.LEFT
                # detect the falling direction
                else:
                    result = self.RIGHT

        # Prune old elements from smoothing history
        self.smoothing_list = list(filter(
            lambda x: x[0] > rospy.Time.now() - rospy.Duration(self.smoothing),
            self.smoothing_list))

        # Add the current element
        self.smoothing_list.append((rospy.Time.now(), result))

        # List only including the results not the whole tuples
        results_list = list(zip(*self.smoothing_list))[1]

        # Check if stable is not in the list otherwise say we are stable
        # This smooths the output but prevents the output of stable when jittering between e.g. right and front
        if self.STABLE in results_list:
            result = self.STABLE

        return result

    def calc_fall_quantification(self, falling_threshold_orientation, falling_threshold_gyro, current_axis_euler,
                                 current_axis__gyro):
        # check if you are moving forward or away from the perpendicular position, by comparing the signs.
        moving_more_upright = numpy.sign(current_axis_euler) != numpy.sign(current_axis__gyro)

        # Check if the orientation is over the point of no return threshold
        over_point_of_no_return = abs(current_axis_euler) > falling_threshold_orientation

        # Calculate quantification if we are moving away from our upright position or if we are over the point of no return
        if not moving_more_upright or over_point_of_no_return:
            # calculatiung the orentation skalar for the threshold
            skalar = max((falling_threshold_orientation - abs(current_axis_euler)) / falling_threshold_orientation, 0)
            # checking if the rotation velocity is lower than the the threshold
            if falling_threshold_gyro * skalar < abs(current_axis__gyro):
                # returning the fall quantification function
                return abs(current_axis__gyro) * (1 - skalar)
        return 0

    def fit(self, x, y):
        # we have to do nothing, as we are not actually fitting any model
        rospy.logwarn_once("You can not train this type of classifier")
        pass

    def score(self, X, y, sample_weight=None):
        return accuracy_score(y, self.predict(X), sample_weight=sample_weight)

    def predict(self, x):
        # only take gyro and orientation from data
        y = []
        for entry in x:
            prediction = self.check_falling(entry[3:6], entry[6:10])
            y.append(prediction)
        return y

    def check_fallen(self, quaternion, not_much_smoothed_gyro):
        """Check if the robot has fallen and is lying on the floor. Returns animation to play, if necessary."""

        # Convert quaternion to fused angles
        fused_roll, fused_pitch, _ = self.fused_from_quat(quaternion)

        # Decides which side is facing downwards.
        if fused_pitch > math.radians(45):
            rospy.loginfo("FALLEN TO THE FRONT")
            return self.FRONT

        if fused_pitch < math.radians(-45):
            rospy.loginfo("FALLEN TO THE BACK")
            return self.BACK

        if fused_roll > math.radians(45):
            rospy.loginfo("FALLEN TO THE RIGHT")
            return self.RIGHT

        if fused_roll < math.radians(-45):
            rospy.loginfo("FALLEN TO THE LEFT")
            return self.LEFT

        # If no side is facing downwards, the robot is not fallen yet.
        return None

    def fused_from_quat(self, q):
        # Fused yaw of Quaternion
        fused_yaw = 2.0 * math.atan2(q[2], q[3])  # Output of atan2 is [-pi,pi], so this expression is in [-2*pi,2*pi]
        if fused_yaw > math.pi:
            fused_yaw -= 2 * math.pi  # fused_yaw is now in[-2 * pi, pi]
        if fused_yaw <= -math.pi:
            fused_yaw += 2 * math.pi  # fused_yaw is now in (-pi, pi]

        # Calculate the fused pitch and roll
        stheta = 2.0 * (q[1] * q[3] - q[0] * q[2])
        sphi = 2.0 * (q[1] * q[2] + q[0] * q[3])
        if stheta >= 1.0:  # Coerce stheta to[-1, 1]
            stheta = 1.0
        elif stheta <= -1.0:
            stheta = -1.0
        if sphi >= 1.0:  # Coerce sphi to[-1, 1]
            sphi = 1.0
        elif sphi <= -1.0:
            sphi = -1.0
        fused_pitch = math.asin(stheta)
        fused_roll = math.asin(sphi)
        return fused_roll, fused_pitch, fused_yaw
