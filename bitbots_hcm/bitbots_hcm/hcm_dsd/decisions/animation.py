from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class Record(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently recording animations
    """

    def perform(self, reevaluate=False):
        if self.blackboard.record_active:
            return "RECORD_ACTIVE"
        else:
            # robot is not recording
            return "FREE"

    def get_reevaluate(self):
        return True


class PlayingExternalAnimation(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently wants to play or plays an animation
    """

    def perform(self, reevaluate=False):
        if self.blackboard.external_animation_running:
            return "ANIMATION_RUNNING"
        else:
            return "FREE"

    def get_reevaluate(self):
        return True
