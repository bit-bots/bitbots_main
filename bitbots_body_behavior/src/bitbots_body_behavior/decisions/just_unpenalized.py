import rospy
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class JustUnpenalized(AbstractDecisionElement):
    def perform(self, reevaluate=False):
        self.publish_debug_data("Seconds since unpenalized", self.blackboard.gamestate.get_seconds_since_unpenalized())
        if self.blackboard.gamestate.get_seconds_since_unpenalized() < 1:
            return 'YES'
        else:
            return 'NO'

    def get_reevaluate(self):
        # Do not reevaluate, should only be reevaluated when the position was reached
        return False
