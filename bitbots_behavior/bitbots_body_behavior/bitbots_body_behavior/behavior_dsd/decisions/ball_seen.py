from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class BallSeen(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Determines whether we are confident regarding the ball's position.
        :param reevaluate:
        :return:
        """
        if self.blackboard.world_model.ball_has_been_seen():
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
