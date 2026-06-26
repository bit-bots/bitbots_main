from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class RecentWalkingGoals(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently walking or just balancing in place.
    """

    def perform(self, reevaluate=False):
        now = self.blackboard.node.get_clock().now().nanoseconds / 1e9

        # Walk node has never sent goals → not walking
        if self.blackboard.last_walking_goal_time is None:
            return "NOT_WALKING"

        time_since_walk_goal = now - self.blackboard.last_walking_goal_time.nanoseconds / 1e9
        self.publish_debug_data("Last Walking Goal Time Delta", time_since_walk_goal)

        # Walk node stopped entirely → not walking
        if time_since_walk_goal >= 0.1:
            return "NOT_WALKING"

        # Walk node is active — check whether significant motion was commanded recently
        if self.blackboard.last_significant_walk_motion_time is None:
            return "NOT_WALKING"

        time_since_motion = now - self.blackboard.last_significant_walk_motion_time.nanoseconds / 1e9
        self.publish_debug_data("Time Since Significant Walk Motion", time_since_motion)

        if time_since_motion < self.blackboard.standing_transition_delay:
            return "STAY_WALKING"
        else:
            return "NOT_WALKING"

    def get_reevaluate(self):
        return True
