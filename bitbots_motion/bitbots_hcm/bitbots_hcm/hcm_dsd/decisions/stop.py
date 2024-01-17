from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement

class Stop(AbstractHCMDecisionElement):
    """
    Handles manual stops
    """

    def perform(self, reevaluate=False):
        if self.blackboard.stopped:
            # we do an action sequence to go into stop and to stay there
            return "STOPPED"
        else:
            return "FREE"

    def get_reevaluate(self):
        return True
