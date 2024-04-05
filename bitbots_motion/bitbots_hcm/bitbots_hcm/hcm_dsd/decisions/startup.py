from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class StartHCM(AbstractHCMDecisionElement):
    """
    Initializes HCM.
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.is_initial = True

    def perform(self, reevaluate=False):
        if self.is_initial:
            self.is_initial = False
            return "START_UP"
        return "RUNNING"

    def get_reevaluate(self):
        return True
