from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class WaitingForFirstKick(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.strikerKicked = False

    def perform(self, reevaluate=False):
        """
        Determines whether we are waiting for the first kick
        :param reevaluate:
        :return:
        """

        self.strikerKicked = (
            self.strikerKicked or self.blackboard.team_data.is_team_mate_kicking()
        )
        if self.blackboard.team_data.role == "striker" or self.strikerKicked:
            return False
        return True

    def get_reevaluate(self):
        return True
