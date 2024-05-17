from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class GoalieHandlingBall(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        It is determined if the goalie is currently going towards the ball
        """
        if self.blackboard.team_data.is_goalie_handling_ball():
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True
