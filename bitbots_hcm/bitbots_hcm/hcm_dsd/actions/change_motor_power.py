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
                blackboard.node.wait_for_service('/core/switch_power', timeout=10)
            except:
                self.get_logger().warn("HCM waiting for switch power service")
            blackboard.node.wait_for_service('/core/switch_power')
            self.switch_power = self.create_client(SetBool, '/core/switch_power')

    def perform(self, reevaluate=False):
        raise NotImplementedError


class TurnMotorsOn(AbstractChangeMotorPower):
    def perform(self, reevaluate=False):
        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            self.switch_power(True)
        return self.pop()


class TurnMotorsOff(AbstractChangeMotorPower):
    def perform(self, reevaluate=False):
        if not self.blackboard.visualization_active and not self.blackboard.simulation_active:
            self.switch_power(False)
        return self.pop()
