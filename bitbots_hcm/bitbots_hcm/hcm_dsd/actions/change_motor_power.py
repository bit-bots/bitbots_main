from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from std_srvs.srv import SetBool


class AbstractChangeMotorPower(AbstractActionElement):
    """
    Switches motor power using the service call of the hardware interface.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters=None)

        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            # In visualization and simulation, we cannot disable motors
            try:
                blackboard.motor_switch_service.wait_for_service(timeout_sec=10)
            except:
                self.blackboard.node.get_logger().warn("HCM waiting for switch power service")
            blackboard.motor_switch_service.wait_for_service()

    def perform(self, reevaluate=False):
        raise NotImplementedError


class TurnMotorsOn(AbstractChangeMotorPower):
    def perform(self, reevaluate=False):
        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            s = SetBool()
            s.Request.data = False
            self.blackboard.motor_switch_service.call(s)
        return self.pop()


class TurnMotorsOff(AbstractChangeMotorPower):
    def perform(self, reevaluate=False):
        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            s = SetBool()
            s.Request.data = True
            self.blackboard.motor_switch_service.call(s)
        return self.pop()
