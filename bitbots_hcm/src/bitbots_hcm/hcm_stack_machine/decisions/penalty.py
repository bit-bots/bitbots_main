from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
import bitbots_hcm.hcm_stack_machine.hcm_connector

class Penalty(AbstractDecisionElement):
    """
    Initializes HCM
    """

    def __init__(self, connector, _):
        super(Penalty, self).__init__(connector)

    def perform(self, connector, reevaluate=False):        
        if connector.hcm.is_penalty():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_PENALTY)
            # we do an action sequence to go into penalty and to stay there            
            self.push(StayInPenalty)
            return self.push(PlayPenaltyAnimation)
        else:
            return self.push(MotorOffTimer)

    def get_reevaluate(self):
        return True
