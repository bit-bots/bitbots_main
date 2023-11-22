from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class IsPenalized(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Determines if the robot is penalized by the game controller.
        """
        self.publish_debug_data("Seconds since unpenalized", self.blackboard.gamestate.get_seconds_since_unpenalized())
        if self.blackboard.gamestate.get_is_penalized():
            return "YES"
        elif self.blackboard.gamestate.get_seconds_since_unpenalized() < 1:
            self.publish_debug_data("Reason", "Just unpenalized")
            return "JUST_UNPENALIZED"
        else:
            return "NO"

    def get_reevaluate(self):
        return True
