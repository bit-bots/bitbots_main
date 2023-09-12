from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement

from humanoid_league_msgs.msg import RobotControlState


class Walking(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently walking
    """

    def perform(self, reevaluate=False):
        if self.blackboard.node.get_clock().now().nanoseconds / 1e9 - self.blackboard.last_walking_goal_time.nanoseconds / 1e9 < 0.1:
            self.blackboard.current_state = RobotControlState.WALKING
            if self.blackboard.animation_requested:
                self.blackboard.animation_requested = False
                # we are walking but we have to stop to play an animation
                return "STOP_WALKING"
            else:
                # we are walking and can stay like this
                return "STAY_WALKING"
        else:
            return "NOT_WALKING"

    def get_reevaluate(self):
        return True