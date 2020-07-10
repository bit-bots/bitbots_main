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
        self.last_pressures = None

    def perform(self, reevaluate=False):
        self.dsd.set_do_not_reevaluate()
        found_same = False
        print(self.last_pressures)
        if self.last_pressures is not None:
            for i in range(len(self.blackboard.pressures)):
                if self.blackboard.pressures[i] == self.last_pressures[i]:
                    found_same = True
                    break
        else:
            found_same = True
        print(found_same)
        print(self.blackboard.last_pressure_update_time )
        print(self.blackboard.last_pressure_update_time.to_sec())
        if not found_same and self.blackboard.last_pressure_update_time and \
                self.blackboard.current_time.to_sec() - self.blackboard.last_pressure_update_time.to_sec() < self.blackboard.pressure_timeout_duration:
            rospy.logwarn("HCM has pressure sensor connection, will now resume.")
            return self.pop()

        else:
            rospy.logwarn_throttle(10, "HCM gets no pressure data. Waiting for pressure sensors to connect. Please"
                                       "check also if the pressure sensors are correctly zeroed. If you "
                                      "have no pressure sensors installed, you may want to set the HCM config "
                                      "accordingly. If you are simulating on your computer you may want to start "
                                      "the HCM with sim:=true.")

        self.last_pressures = self.blackboard.pressures


class WaitForMotors(AbstractActionElement):
    """
    Waits for the motors to connect and publishes warnings while doing so
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(WaitForMotors, self).__init__(blackboard, dsd, parameters)
        self.last_positions = None

    def perform(self, reevaluate=False):
        self.dsd.set_do_not_reevaluate()
        if self.last_positions is not None and self.blackboard.current_joint_positions.position == self.last_positions:
            rospy.logwarn("HCM has motor connection, will now resume.")
            return self.pop()
        else:
            rospy.logwarn_throttle(10   ,
                                   "HCM gets no data from the motors (/joint_states). Waiting for the motors to connect.")

        self.last_position = self.blackboard.current_joint_positions.position
