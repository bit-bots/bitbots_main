from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement


class WaitForIMUStartup(AbstractHCMActionElement):
    """
    Waits for the IMU to connect and does not complain as we are still in start up.
    """

    def perform(self, reevaluate=False):
        pass


class WaitForIMU(AbstractHCMActionElement):
    """
    Waits for the IMU to connect and publishes warnings while doing so
    """

    def perform(self, reevaluate=False):
        self.blackboard.node.get_logger().warn(
            "HCM gets no IMU data. Waiting for IMU to connect.", throttle_duration_sec=10
        )


class WaitForMotors(AbstractHCMActionElement):
    """
    Waits for the motors to connect and publishes warnings while doing so
    """

    def perform(self, reevaluate=False):
        self.blackboard.node.get_logger().warn(
            "HCM gets no data from the motors (/joint_states). Waiting for the motors to connect.",
            throttle_duration_sec=10,
        )
