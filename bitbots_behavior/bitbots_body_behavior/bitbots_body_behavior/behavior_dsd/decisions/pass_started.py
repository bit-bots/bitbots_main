from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class PassStarted(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if self.blackboard.team_data.is_team_mate_kicking():
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True
