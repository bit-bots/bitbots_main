from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement

from humanoid_league_msgs.msg import RobotControlState


class RecentWalkingGoals(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently getting joint commands to from the walking node
    """

    def perform(self, reevaluate=False):
        if self.blackboard.node.get_clock().now().nanoseconds / 1e9 - self.blackboard.last_walking_goal_time.nanoseconds / 1e9 < 0.1:
            # we are walking and can stay like this
            return "STAY_WALKING"
        else:
            return "NOT_WALKING"

    def get_reevaluate(self):
        return True