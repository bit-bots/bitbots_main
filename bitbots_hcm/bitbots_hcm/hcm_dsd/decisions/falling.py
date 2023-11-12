import math
from enum import Enum

import numpy as np
from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement
from bitbots_utils.transforms import quat2fused
from rclpy.duration import Duration


class FallDirection(Enum):
    STABLE = 0
    FRONT = 1
    BACK = 2
    LEFT = 3
    RIGHT = 4


class Falling(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently falling and has to act on this
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

        # Get parameters
        self.thresh_gyro_pitch = self.blackboard.node.get_parameter("falling_thresh_gyro_pitch").value
        self.thresh_gyro_roll = self.blackboard.node.get_parameter("falling_thresh_gyro_roll").value
        self.thresh_orient_pitch = math.radians(self.blackboard.node.get_parameter("falling_thresh_orient_pitch").value)
        self.thresh_orient_roll = math.radians(self.blackboard.node.get_parameter("falling_thresh_orient_roll").value)
        self.smoothing = self.blackboard.node.get_parameter("smooth_threshold").value

        # Initialize smoothing list that stores the last results
        self.smoothing_list: list[FallDirection] = []



    def perform(self, reevaluate=False):
        """Checks if the robot is currently falling and in which direction."""
        # Check if detection is active
        if not self.blackboard.falling_detection_active:
            return "NOT_FALLING"

        # Get angular from the IMU
        angular_velocity = self.blackboard.gyro

        # Convert orientation to fused angles
        fused_roll, fused_pitch, _, _ = quat2fused(self.blackboard.quaternion, order="xyzw")

        # setting the fall quantification function
        roll_fall_quantification = self.calc_fall_quantification(
            self.thresh_orient_roll,
            self.thresh_gyro_roll,
            fused_roll,
            angular_velocity[0])

        pitch_fall_quantification = self.calc_fall_quantification(
            self.thresh_orient_pitch,
            self.thresh_gyro_pitch,
            fused_pitch,
            angular_velocity[1])

        if roll_fall_quantification + pitch_fall_quantification == 0:
            result = FallDirection.STABLE
        else:
            # compare quantification functions
            if pitch_fall_quantification > roll_fall_quantification:
                # detect the falling direction
                if fused_pitch < 0:
                    result = FallDirection.BACK
                # detect the falling direction
                else:
                    result = FallDirection.FRONT
            else:
                # detect the falling direction
                if fused_roll < 0:
                    result = FallDirection.LEFT
                # detect the falling direction
                else:
                    result = FallDirection.RIGHT

        # Prune old elements from smoothing history
        self.smoothing_list = list(filter(
            lambda x: x[0] > self.blackboard.node.get_clock().now() - Duration(seconds=self.smoothing),
            self.smoothing_list))

        # Add the current element
        self.smoothing_list.append((self.blackboard.node.get_clock().now(), result))

        # List only including the results not the whole tuples
        results_list = list(zip(*self.smoothing_list))[1]

        # Check if stable is not in the list otherwise say we are stable
        # This smooths the output but prevents the output of stable when jittering between e.g. right and front
        if FallDirection.STABLE in results_list:
            result = FallDirection.STABLE

        # Return the appropriate result
        if result == FallDirection.STABLE:
            return "NOT_FALLING"
        elif result == FallDirection.FRONT:
            return "FALLING_FRONT"
        elif result == FallDirection.BACK:
            return "FALLING_BACK"
        elif result == FallDirection.LEFT:
            return "FALLING_LEFT"
        elif result == FallDirection.RIGHT:
            return "FALLING_RIGHT"
        else:
            raise ValueError("Unknown falling direction")

    def calc_fall_quantification(self, falling_threshold_orientation, falling_threshold_gyro, current_axis_euler,
                                 current_axis__gyro):
        # check if you are moving forward or away from the perpendicular position, by comparing the signs.
        moving_more_upright = np.sign(current_axis_euler) != np.sign(current_axis__gyro)

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

    def get_reevaluate(self):
        return True
