import rospy
import humanoid_league_msgs.msg
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_dsd.hcm_blackboard
from bitbots_hcm.hcm_dsd.hcm_blackboard import STATE_HARDWARE_PROBLEM
from std_srvs.srv import SetBool


class AbstractChangeMotorPower(AbstractActionElement):
    """
    Switches motor power using the service call of the hardware interface.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractChangeMotorPower, self).__init__(blackboard, dsd, parameters=None)

        try:
            rospy.wait_for_service('/core/switch_power', timeout=10)
        except:
            rospy.logwarn("HCM waiting for switch power service")
        rospy.wait_for_service('/core/switch_power')
        self.switch_power = rospy.ServiceProxy('/core/switch_power', SetBool)

    def perform(self, reevaluate=False):
        raise NotImplementedError


class TurnMotorsOn(AbstractChangeMotorPower):
    def perform(self, reevaluate=False):
        self.switch_power(True)
        return self.pop()


class TurnMotorsOff(AbstractChangeMotorPower):
    def perform(self, reevaluate=False):
        self.switch_power(False)
        return self.pop()
