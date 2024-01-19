from bitbots_localization_handler.localization_dsd.decisions import AbstractLocalizationDecisionElement
from bitbots_msgs.msg import RobotControlState


class CheckPickup(AbstractLocalizationDecisionElement):
    """
    Checks if robot is picked up
    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if self.blackboard.robot_control_state == RobotControlState.PICKED_UP:
            self.blackboard.last_state_pickup = True
            return "UP"
        else:
            if self.blackboard.last_state_pickup:
                self.blackboard.last_state_pickup = False
                return "JUST_DOWN"

        return "DOWN"

    def get_reevaluate(self):
        return True
