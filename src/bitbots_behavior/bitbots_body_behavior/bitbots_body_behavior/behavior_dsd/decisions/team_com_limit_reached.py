from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class TeamComLimitReached(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if self.blackboard.gamstate.get_team_com_limit_has_reached():
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True
