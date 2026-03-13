from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class WhistleDetected(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.previous_timestep_whistle_detected = self.blackboard.gamestate.last_timestep_whistle_detected

    def perform(self, reevaluate=False):
        """
        Returns "DETECTED" if a whistle was detected while the decision element was active, as part of the
        DSD stack. If we have detected a whistle previously, but not during this decision element being on
        the decision stack it counts as "NOT_DETECTED".
        """
        if self.previous_timestep_whistle_detected == self.blackboard.gamestate.last_timestep_whistle_detected:
            return "NOT_DETECTED"

        return "DETECTED"

    def get_reevaluate(self):
        return True
