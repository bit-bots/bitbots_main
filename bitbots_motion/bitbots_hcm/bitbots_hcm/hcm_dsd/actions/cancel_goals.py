from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement


class CancelGoals(AbstractHCMActionElement):
    """
    Cancels all animation, dynup and move_base goals
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self):
        if self.blackboard.animation_action_current_goal is not None:
            try:
                self.blackboard.animation_action_current_goal.cancel()
            except Exception as e:
                self.blackboard.node.get_logger().error("Could not cancel animation goal: " + str(e))
        if self.blackboard.dynup_action_current_goal is not None:
            try:
                self.blackboard.dynup_action_current_goal.cancel()
            except Exception as e:
                self.blackboard.node.get_logger().error("Could not cancel dynup goal: " + str(e))

        self.blackboard.cancel_path_planning()
        self.pop()
