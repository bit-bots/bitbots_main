from bitbots_localization_handler.localization_dsd.decisions import AbstractLocalizationDecisionElement
from bitbots_msgs.msg import RobotControlState


class CheckPickup(AbstractLocalizationDecisionElement):
    """
    Checks if robot is picked up
    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if self.blackboard.picked_up():
            return "UP"
        return "DOWN"

    def get_reevaluate(self):
        return True
