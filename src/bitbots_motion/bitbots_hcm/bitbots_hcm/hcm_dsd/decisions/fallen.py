import math

import numpy as np
from bitbots_utils.transforms import quat2fused

from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class Fallen(AbstractHCMDecisionElement):
    """
    Decides if the robot is fallen and lying on the ground
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

        # Get parameters
        self.fallen_orientation_thresh = math.radians(
            self.blackboard.node.get_parameter("fallen_orientation_thresh").value
        )
        self.fallen_angular_velocity_thresh = self.blackboard.node.get_parameter("fallen_angular_velocity_thresh").value

        # publishes if robot is fallen
        self.is_fallen_publisher = self.create_publisher(
            bool, "hsl_gamecontroller/is_fallen", 1
        )


    def perform(self, reevaluate=False):
        # Check of the fallen detection is active
        if not self.blackboard.is_stand_up_active:
            self.publish_if_fallen(False)
            return "NOT_FALLEN"

        # Get angular velocity from the IMU
        angular_velocity = self.blackboard.gyro

        # Check if the robot is rotating
        if np.mean(np.abs(angular_velocity)) >= 0.2:
            self.publish_if_fallen(False)
            return "NOT_FALLEN"

        # Convert quaternion to fused angles
        fused_roll, fused_pitch, _, _ = quat2fused(self.blackboard.quaternion, order="xyzw")

        # Decides which side is facing downwards.
        if fused_pitch > self.fallen_orientation_thresh:
            self.publish_if_fallen(True)
            return "FALLEN_FRONT"

        if fused_pitch < -self.fallen_orientation_thresh:
            self.publish_if_fallen(True)
            return "FALLEN_BACK"

        if fused_roll > self.fallen_orientation_thresh:
            self.publish_if_fallen(True)
            return "FALLEN_RIGHT"

        if fused_roll < -self.fallen_orientation_thresh:
            self.publish_if_fallen(True)
            return "FALLEN_LEFT"

        self.publish_if_fallen(False)
        return "NOT_FALLEN"
    
    def publish_if_fallen(self, is_fallen):
        # publishes if robot is fallen
        self.is_fallen_publisher.publish(is_fallen)

    def get_reevaluate(self):
        return True
