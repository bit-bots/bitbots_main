from std_srvs.srv import SetBool

from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement
from livelybot_power.msg import PowerSwitch

class AbstractChangeMotorPower(AbstractHCMActionElement):
    """
    Switches motor power using the service call of the hardware interface.
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        raise NotImplementedError


class TurnMotorsOff(AbstractChangeMotorPower):
    def perform(self, reevaluate=False):
        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            msg = PowerSwitch()
            msg.power_switch = 0
            self.blackboard.motor_switch_pub.pub(msg)
        return self.pop()
