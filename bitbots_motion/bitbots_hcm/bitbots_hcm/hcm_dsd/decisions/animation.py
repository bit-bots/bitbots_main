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
        # Check if the robot is currently playing an animation
        if not self.blackboard.external_animation_running:
            return "FREE"

        self.blackboard.node.get_logger().info(str(self.blackboard.last_animation_goal_time))

        # We can safely assume that the last animation start time is set because the animation is running
        # Calculate time since last animation start
        time_delta_since_last_animation_start = (
            self.blackboard.node.get_clock().now().nanoseconds / 1e9
            - self.blackboard.last_animation_start_time.nanoseconds / 1e9
        )
        # If the time since the last animation start is less than 0.5 seconds the animation just started and we might not have received the first command yet
        if time_delta_since_last_animation_start < 0.5:
            return "ANIMATION_RUNNING"

        # If the animation is running for more longer we check if the last goal was more than 0.5 seconds ago and abort the animation if it was
        # Check if the last animation goal was more than 0.5 seconds ago
        if (
            self.blackboard.last_animation_goal_time is None
            or (
                self.blackboard.node.get_clock().now().nanoseconds / 1e9
                - self.blackboard.last_animation_goal_time.nanoseconds / 1e9
            )
            > 0.5
        ):
            return "ANIMATION_SERVER_TIMEOUT"

        # We are recieving goals and the animation is running
        return "ANIMATION_RUNNING"

    def get_reevaluate(self):
        return True
