from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class CurrentScore(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        own_goals = self.blackboard.gamestate.get_own_goals()
        opp_goals = self.blackboard.gamestate.get_opp_goals()

        if own_goals == opp_goals:
            return "DRAW"
        elif own_goals > opp_goals:
            return "AHEAD"
        else:
            return "BEHIND"

    def get_reevaluate(self):
        return True
