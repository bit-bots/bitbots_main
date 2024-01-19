from bitbots_localization_handler.localization_dsd.decisions import AbstractLocalizationDecisionElement
from bitbots_msgs.msg import RobotControlState


class GettingUpState(AbstractLocalizationDecisionElement):
    """
    Checks if the robot falls, stands up or is freshly standing
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.get_up_states = [RobotControlState.FALLING, RobotControlState.FALLEN, RobotControlState.GETTING_UP]

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if self.blackboard.robot_control_state in self.get_up_states:
            self.blackboard.last_state_get_up = True
            return "YES"
        else:
            if self.blackboard.last_state_get_up:
                self.blackboard.last_state_get_up = False
                return "GOTUP"

        return "NO"

    def get_reevaluate(self):
        return True
