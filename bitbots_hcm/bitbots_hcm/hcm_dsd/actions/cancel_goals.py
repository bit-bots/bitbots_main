from rclpy.action import ActionClient
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class CancelGoals(AbstractActionElement):

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self):
        if self.blackboard.animation_action_current_goal is not None:
            try:
                self.blackboard.animation_action_current_goal.cancel()
            except:
                pass
        if self.blackboard.dynup_action_current_goal is not None:
            try:
                self.blackboard.dynup_action_current_goal.cancel()
            except:
                pass
            
        self.blackboard.cancel_move_base_goal()
        self.pop()

