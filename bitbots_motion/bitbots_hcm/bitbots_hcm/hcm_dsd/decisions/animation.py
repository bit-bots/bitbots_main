from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class RecordAnimation(AbstractHCMDecisionElement):
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
        # Calculate time since last animation goal
        time_delta = (
            self.blackboard.node.get_clock().now().nanoseconds / 1e9
            - self.blackboard.last_animation_goal_time.nanoseconds / 1e9
        )

        # Log the time delta between now and the last animation goal
        self.publish_debug_data("Last Animation Goal Time Delta", time_delta)

        # Check if the robot is currently playing an animation
        if self.blackboard.external_animation_running:
            if time_delta < 0.1:
                return "ANIMATION_RUNNING"
            else:
                return "ANIMATION_SERVER_TIMEOUT"
        else:
            return "FREE"

    def get_reevaluate(self):
        return True
