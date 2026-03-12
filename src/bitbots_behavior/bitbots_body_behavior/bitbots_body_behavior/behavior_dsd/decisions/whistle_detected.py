from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class WhistleDetected(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard = blackboard
        self.last_timestep_whistle_detected = self.blackboard.last_timestep_whistle_detected

    def perform(self, reevaluate=False):
        if self.last_timestep_whistle_detected == self.blackboard.last_timestep_whistle_detected: 
            return "NOT_DETECTED"
        return "DETECTED"

    def get_reevaluate(self):
        return True
