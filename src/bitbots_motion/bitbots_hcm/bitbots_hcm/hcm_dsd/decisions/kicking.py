from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class KickRequested(AbstractHCMDecisionElement):
    """
    Decides if the behavior currently requests the (RL) kick motion.

    The behavior requests/stops the kick on the ``rl_kick_active`` topic. While requested, the HCM
    stays in the KICKING robot state, which is what gates the RL kick node.
    """

    def perform(self, reevaluate=False):
        self.publish_debug_data("Kick requested", self.blackboard.kick_requested)
        if self.blackboard.kick_requested:
            return "KICKING"
        return "NOT_KICKING"

    def get_reevaluate(self):
        return True
