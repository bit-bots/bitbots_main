from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class InSquat(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently recording animations
    """

    def perform(self, reevaluate=False):
        if self.blackboard.in_squat:
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True
