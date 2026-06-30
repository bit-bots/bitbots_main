from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class AcrobaticMotionRunning(AbstractHCMDecisionElement):
    """Checks if an acrobatic motion (e.g. cartwheel) has been requested via the perform_acrobatic_mode service."""

    def perform(self, reevaluate=False):
        if self.blackboard.acrobatic_motion_running:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
