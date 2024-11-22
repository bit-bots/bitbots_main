from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class ConfigRole(AbstractDecisionElement):
    """
    Decides what kind of behaviour the robot performs
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.role = self.blackboard.team_data.role

    def perform(self, reevaluate=False):
        assert self.role in self.blackboard.config["roles"], "No valid role specified"
        return self.role.upper()

    def get_reevaluate(self):
        """
        The role does not change during the game
        """
        return False
