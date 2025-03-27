from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class own_team_is_in_possession(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        team_in_possession = self.blackboard.world_model.team_nearest_to_ball()

        if team_in_possession == 1:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
