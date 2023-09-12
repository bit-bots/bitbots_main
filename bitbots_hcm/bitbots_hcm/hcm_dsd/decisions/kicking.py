from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement

from humanoid_league_msgs.msg import RobotControlState


class Kicking(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently kicking
    """

    def perform(self, reevaluate=False):
        if self.blackboard.last_kick_feedback is not None and \
                (self.blackboard.node.get_clock().now() - self.blackboard.last_kick_feedback).nanoseconds / 1e9 < 0.2:
            self.blackboard.current_state = RobotControlState.KICKING
            return 'KICKING'
        else:
            return 'NOT_KICKING'

    def get_reevaluate(self):
        return True