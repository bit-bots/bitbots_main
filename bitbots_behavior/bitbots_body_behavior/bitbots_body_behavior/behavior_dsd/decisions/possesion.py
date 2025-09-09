from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class OwnTeamIsInPossession(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        own_ball_distance = self.blackboard.world_model.distance_to_ball_own_team()
        opponent_ball_distance = self.blackboard.world_model.distance_to_ball_opponent_team()
        if own_ball_distance < opponent_ball_distance:
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True
