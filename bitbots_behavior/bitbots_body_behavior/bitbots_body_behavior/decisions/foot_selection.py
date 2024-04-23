from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class FootSelection(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Determines which foot should be used to kick the ball
        """

        if self.blackboard.world_model.get_ball_position_uv()[1] < 0:
            return "RIGHT"
        return "LEFT"

    def get_reevaluate(self):
        return True
