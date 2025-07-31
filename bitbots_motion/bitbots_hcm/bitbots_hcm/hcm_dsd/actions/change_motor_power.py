from std_srvs.srv import SetBool

from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement


class AbstractChangeMotorPower(AbstractHCMActionElement):
    """
    Switches motor power using the service call of the hardware interface.
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

        # In visualization and simulation, we cannot disable motors
        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            if not self.blackboard.motor_switch_service.wait_for_service(timeout_sec=10):
                self.blackboard.node.get_logger().warn("HCM waiting for switch power service")
            self.blackboard.motor_switch_service.wait_for_service()

    def perform(self, reevaluate=False):
        raise NotImplementedError


class TurnMotorsOff(AbstractChangeMotorPower):
    def perform(self, reevaluate=False):
        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            req = SetBool.Request()
            req.data = False
            self.blackboard.motor_switch_service.call_async(req)
        return self.pop()
