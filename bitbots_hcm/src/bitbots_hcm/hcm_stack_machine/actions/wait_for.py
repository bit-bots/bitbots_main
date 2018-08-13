import rospy 
import humanoid_league_msgs.msg
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_stack_machine.hcm_connector


class WaitForIMU(AbstractActionElement):
    """
    Waits for the IMU to connect and publishes warnings while doing so
    """

    def perform(self, connector, reevaluate=False):        
        if connector.is_imu_timeout():
            rospy.logwarn_throttle(5, "HCM gets no IMU data. Will now wait for IMU to connect.")
        else:
            rospy.logwarn("HCM has IMU connection, will now resume.")            
            return self.pop()

class WaitForMotors(AbstractActionElement):
    """
    Waits for the motors to connect and publishes warnings while doing so
    """

    def perform(self, connector, reevaluate=False):        
        if connector.hcm.are_motors_active():
            rospy.logwarn("HCM has connection, will now resume.")            
            return self.pop()
        else:
            rospy.logwarn_throttle(1, "HCM gets no data from the motors. Will now wait for the motors to connect.")