from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class WaitingForFirstKick(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.strikerKicked = False
        self.role = self.blackboard.team_data.role
        self.role_position = self.blackboard.misc.position_number

    def perform(self, reevaluate=False):
        """
        Determines whether we are waiting for the first kick
        :param reevaluate:
        :return:
        """
        self.publish_debug_data("strikerKicked", self.strikerKicked)
        self.publish_debug_data("role", self.role)
        self.publish_debug_data("role_position", self.role_position)
        self.strikerKicked = (
            self.strikerKicked or self.blackboard.team_data.is_team_mate_kicking()
        )
        if (self.role == "offense" and self.role_position == 0) or self.strikerKicked:
            return "NO"
        return "YES"

    def get_reevaluate(self):
        return True
