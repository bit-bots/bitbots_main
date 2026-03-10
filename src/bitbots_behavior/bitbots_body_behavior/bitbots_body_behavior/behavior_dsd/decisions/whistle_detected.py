from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class WhistleDetected(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.bb = blackboard
        self.bb.game_state_capsule.whistle_detected = False

    def perform(self, reevaluate=False):
        if self.bb.game_state_capsule.whistle_detected:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return False
