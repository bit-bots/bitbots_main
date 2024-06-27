from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class ForgetBall(AbstractActionElement):
    blackboard: BodyBlackboard

    def perform(self, reevaluate=False):
        self.blackboard.world_model.forget_ball()
        self.pop()
