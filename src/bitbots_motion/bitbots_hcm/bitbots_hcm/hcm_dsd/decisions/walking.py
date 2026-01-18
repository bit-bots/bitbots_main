from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class RecentWalkingGoals(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently getting joint commands to from the walking node
    """

    def perform(self, reevaluate=False):
        # Check if we have received a walking goal at all
        if self.blackboard.last_walking_goal_time is None:
            return "NOT_WALKING"

        # Calculate the time delta between now and the last walking goal
        time_delta = (
            self.blackboard.node.get_clock().now().nanoseconds / 1e9
            - self.blackboard.last_walking_goal_time.nanoseconds / 1e9
        )

        # Log the time delta between now and the last walking goal
        self.publish_debug_data("Last Walking Goal Time Delta", time_delta)

        # If the time delta is smaller enough, we are still walking
        if time_delta < 0.1:
            # we are walking and can stay like this
            return "STAY_WALKING"
        else:
            return "NOT_WALKING"

    def get_reevaluate(self):
        return True
