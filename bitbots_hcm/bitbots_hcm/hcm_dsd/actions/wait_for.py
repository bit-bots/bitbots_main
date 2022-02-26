import rclpy
from rclpy.node import Node
import humanoid_league_msgs.msg
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_dsd.hcm_blackboard


class WaitForIMUStartup(AbstractActionElement):
    """
    Waits for the IMU to connect and does not complain as we are still in start up.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        pass


class WaitForIMU(AbstractActionElement):
    """
    Waits for the IMU to connect and publishes warnings while doing so
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.get_logger().warn("HCM gets no IMU data. Waiting for IMU to connect.", throttle_duration_sec=10


class WaitForPressureStartup(AbstractActionElement):
    """
    Waits for the pressure sensors to connect and not complain since we are still starting up.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        pass


class WaitForPressure(AbstractActionElement):
    """
    Waits for the pressure sensors to connect and publishes warnings while doing so
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.node.logwarn_throttle(30,
                                         "HCM gets no correct pressure data. Waiting for pressure sensors to connect.\n"
                                         "Use rqt_monitor to check hardware status. "
                                         "Please check if the pressure sensors are correctly zeroed. If you "
                                         "have no pressure sensors installed, you may want to set the HCM config "
                                         "accordingly. If you just running a visualization on your computer you may want to "
                                         "set the visualization_active parameter to True.")


class WaitForMotorStartup(AbstractActionElement):
    """
    Waits for the motors on startup without complaining if it takes a moment.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        pass


class WaitForMotors(AbstractActionElement):
    """
    Waits for the motors to connect and publishes warnings while doing so
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.node.logwarn_throttle(10,
                                         "HCM gets no data from the motors (/joint_states). Waiting for the motors to "
                                         "connect.")
