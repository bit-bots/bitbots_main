from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement


class CancelGoals(AbstractHCMActionElement):
    """
    Cancels all animation and move_base goals
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self):
        if self.blackboard.animation_action_current_goal is not None:
            try:
                self.blackboard.animation_action_current_goal.cancel()
            except Exception as e:
                self.blackboard.node.get_logger().error("Could not cancel animation goal: " + str(e))

        self.blackboard.cancel_path_planning()
        self.pop()
