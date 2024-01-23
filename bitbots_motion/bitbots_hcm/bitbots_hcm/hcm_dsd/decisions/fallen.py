import math

import numpy as np
from bitbots_utils.transforms import quat2fused

from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class Fallen(AbstractHCMDecisionElement):
    """
    Decides if the robot is fallen and lying on the ground
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

        # Get parameters
        self.fallen_orientation_thresh = math.radians(
            self.blackboard.node.get_parameter("fallen_orientation_thresh").value
        )
        self.fallen_angular_velocity_thresh = self.blackboard.node.get_parameter("fallen_angular_velocity_thresh").value

    def perform(self, reevaluate=False):
        # Check of the fallen detection is active
        if not self.blackboard.is_stand_up_active:
            return "NOT_FALLEN"

        # Get angular velocity from the IMU
        angular_velocity = self.blackboard.gyro

        # Check if the robot is rotating
        if np.mean(np.abs(angular_velocity)) >= 0.2:
            return "NOT_FALLEN"

        # Convert quaternion to fused angles
        fused_roll, fused_pitch, _, _ = quat2fused(self.blackboard.quaternion, order="xyzw")

        # Decides which side is facing downwards.
        if fused_pitch > self.fallen_orientation_thresh:
            self.blackboard.node.get_logger().info("FALLEN TO THE FRONT")
            return "FALLEN_FRONT"

        if fused_pitch < -self.fallen_orientation_thresh:
            self.blackboard.node.get_logger().info("FALLEN TO THE BACK")
            return "FALLEN_BACK"

        if fused_roll > self.fallen_orientation_thresh:
            self.blackboard.node.get_logger().info("FALLEN TO THE RIGHT")
            return "FALLEN_RIGHT"

        if fused_roll < -self.fallen_orientation_thresh:
            self.blackboard.node.get_logger().info("FALLEN TO THE LEFT")
            return "FALLEN_LEFT"

        return "NOT_FALLEN"

    def get_reevaluate(self):
        return True
