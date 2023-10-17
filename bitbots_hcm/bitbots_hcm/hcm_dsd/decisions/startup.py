import math

from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement

from bitbots_msgs.msg import RobotControlState


class StartHCM(AbstractHCMDecisionElement):
    """
    Initializes HCM.
    """
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.is_initial = True

    def perform(self, reevaluate=False):
        if self.is_initial:
            self.is_initial = False
            return "START_UP"
        return "RUNNING"

    def get_reevaluate(self):
        return True
