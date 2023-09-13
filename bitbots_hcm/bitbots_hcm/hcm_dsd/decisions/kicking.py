from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement

from humanoid_league_msgs.msg import RobotControlState


class RecentKickGoals(AbstractHCMDecisionElement):
    """
    Decides if the kick is currently sending joint commands
    """

    def perform(self, reevaluate=False):
        if self.blackboard.last_kick_feedback is not None and \
                (self.blackboard.node.get_clock().now() - self.blackboard.last_kick_feedback).nanoseconds / 1e9 < 0.2:
            return 'KICKING'
        else:
            return 'NOT_KICKING'

    def get_reevaluate(self):
        return True