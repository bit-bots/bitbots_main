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


class WaitForPressureStartup(AbstractHCMActionElement):
    """
    Waits for the pressure sensors to connect and not complain since we are still starting up.
    """

    def perform(self, reevaluate=False):
        pass


class WaitForPressure(AbstractHCMActionElement):
    """
    Waits for the pressure sensors to connect and publishes warnings while doing so
    """

    def perform(self, reevaluate=False):
        self.blackboard.node.get_logger().warn(
            "HCM gets no correct pressure data. Waiting for pressure sensors to connect.\n"
            "Use rqt_monitor to check hardware status. "
            "Please check if the pressure sensors are correctly zeroed. If you "
            "have no pressure sensors installed, you may want to set the HCM config "
            "accordingly. If you just running a visualization on your computer you may want to "
            "set the visualization_active parameter to True.",
            throttle_duration_sec=30,
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
