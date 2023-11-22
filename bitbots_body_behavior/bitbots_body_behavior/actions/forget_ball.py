from dynamic_stack_decider.abstract_action_element import AbstractActionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class ForgetBall(AbstractActionElement):
    blackboard: BodyBlackboard

    def perform(self, reevaluate=False):
        self.blackboard.world_model.forget_ball(own=True, team=True, reset_ball_filter=True)
        self.pop()
