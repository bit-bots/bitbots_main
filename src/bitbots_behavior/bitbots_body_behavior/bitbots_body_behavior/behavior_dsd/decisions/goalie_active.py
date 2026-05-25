from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class GoalieActive(AbstractDecisionElement):
    """
    Decides whether the goalie is on field or not
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if self.blackboard.gamestate.get_team_com_limit_has_reached():
            if self.blackboard.team_data.get_was_goalie_active():
                return "YES"
            else:
                return "NO"
        else:
            if self.blackboard.team_data.get_is_goalie_active():
                return "YES"
            else:
                return "NO"

    def get_reevaluate(self):
        return True
