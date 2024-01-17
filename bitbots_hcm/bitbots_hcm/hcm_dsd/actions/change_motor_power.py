from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement
from std_srvs.srv import SetBool


class AbstractChangeMotorPower(AbstractHCMActionElement):
    """
    Switches motor power using the service call of the hardware interface.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            # In visualization and simulation, we cannot disable motors
            try:
                self.blackboard.motor_switch_service.wait_for_service(timeout_sec=10)
            except:
                self.blackboard.node.get_logger().warn("HCM waiting for switch power service")
            self.blackboard.motor_switch_service.wait_for_service()

    def perform(self, reevaluate=False):
        raise NotImplementedError


class TurnMotorsOn(AbstractChangeMotorPower):
    def perform(self, reevaluate=False):
        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            req = SetBool.Request()
            req.data = True
            self.blackboard.motor_switch_service.call(req)
        return self.pop()


class TurnMotorsOff(AbstractChangeMotorPower):
    def perform(self, reevaluate=False):
        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            req = SetBool.Request()
            req.data = False
            self.blackboard.motor_switch_service.call(req)
        return self.pop()
