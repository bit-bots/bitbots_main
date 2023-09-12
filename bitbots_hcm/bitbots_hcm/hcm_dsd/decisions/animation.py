from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement

from humanoid_league_msgs.msg import RobotControlState


class Record(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently recording animations
    """

    def perform(self, reevaluate=False):
        if self.blackboard.record_active:
            self.blackboard.current_state = RobotControlState.RECORD
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
            self.blackboard.current_state = RobotControlState.ANIMATION_RUNNING
            return "ANIMATION_RUNNING"
        else:
            return "FREE"

    def get_reevaluate(self):
        return True
