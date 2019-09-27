import rospy
import humanoid_league_msgs.msg
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_dsd.hcm_blackboard


class WaitForIMU(AbstractActionElement):
    """
    Waits for the IMU to connect and publishes warnings while doing so
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(WaitForIMU, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if not self.blackboard.last_imu_update_time or self.blackboard.current_time.to_sec() - self.blackboard.last_imu_update_time.to_sec() > self.blackboard.imu_timeout_duration:
            rospy.logwarn_throttle(5, "HCM gets no IMU data. Waiting for IMU to connect.")
        else:
            rospy.logwarn("HCM has IMU connection, will now resume.")
            return self.pop()


class WaitForPressure(AbstractActionElement):
    """
    Waits for the pressure sensors to connect and publishes warnings while doing so
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(WaitForPressure, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if not self.blackboard.last_pressure_update_time or self.blackboard.current_time.to_sec() - self.blackboard.last_pressure_update_time.to_sec() > self.blackboard.pressure_timeout_duration:
            rospy.logwarn_throttle(5, "HCM gets no pressure data. Waiting for pressure sensors to connect. If you "
                                      "have no pressure sensors installed, you may want to set the HCM config "
                                      "accordingly. If you are simulating on your computer you may want to start "
                                      "the HCM with sim:=true.")
        else:
            rospy.logwarn("HCM has pressure sensor connection, will now resume.")
            return self.pop()


class WaitForMotors(AbstractActionElement):
    """
    Waits for the motors to connect and publishes warnings while doing so
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(WaitForMotors, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if self.blackboard.current_time.to_sec() - self.blackboard.last_motor_update_time.to_sec() < 0.1:
            rospy.logwarn("HCM has motor connection, will now resume.") #TODO this message is never displayed
            return self.pop()
        else:
            rospy.logwarn_throttle(1, "HCM gets no data from the motors (/joint_states). Waiting for the motors to connect.")
