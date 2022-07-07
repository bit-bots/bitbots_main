from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class ForgetBall(AbstractActionElement):
    def perform(self, reevaluate=False):
        self.blackboard.world_model.forget_ball(own=True, team=True, reset_ball_filter=True)
        self.pop()
