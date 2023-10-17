from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement

from bitbots_msgs.msg import RobotControlState


class RecentKickGoals(AbstractHCMDecisionElement):
    """
    Decides if the kick is currently sending joint commands
    """

    def perform(self, reevaluate=False):
        # Check if we have received a kick goal at all
        if self.blackboard.last_kick_goal_time is None:
            return "NOT_KICKING"

        # Calculate the time delta between now and the last kick goal
        time_delta = self.blackboard.node.get_clock().now().nanoseconds / 1e9 - self.blackboard.last_kick_goal_time.nanoseconds / 1e9

        # Log the time delta between now and the last kick goal
        self.publish_debug_data("Last Kick Goal Time Delta", time_delta)

        # If the time delta is smaller enough, we are still kicking
        if time_delta < 0.1:
            # we are walking and can stay like this
            return "KICKING"
        else:
            return "NOT_KICKING"

    def get_reevaluate(self):
        return True
