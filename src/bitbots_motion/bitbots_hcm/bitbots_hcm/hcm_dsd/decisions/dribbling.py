from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class RecentDribbleGoals(AbstractHCMDecisionElement):
    """
    Decides if the dribble policy is currently sending joint commands.

    Works like the walking check below it in the tree: while dribble motor
    goals arrive, the robot is in the DRIBBLING state, which mutes the regular
    walking goals so the dribble policy overrides the walk.
    """

    def perform(self, reevaluate=False):
        # Check if we have received a dribble goal at all
        if self.blackboard.last_dribble_goal_time is None:
            return "NOT_DRIBBLING"

        # Calculate the time delta between now and the last dribble goal
        time_delta = (
            self.blackboard.node.get_clock().now().nanoseconds / 1e9
            - self.blackboard.last_dribble_goal_time.nanoseconds / 1e9
        )

        # Log the time delta between now and the last dribble goal
        self.publish_debug_data("Last Dribble Goal Time Delta", time_delta)

        # If the time delta is small enough, the dribble policy is still active
        if time_delta < 0.1:
            return "DRIBBLING"
        else:
            return "NOT_DRIBBLING"

    def get_reevaluate(self):
        return True
