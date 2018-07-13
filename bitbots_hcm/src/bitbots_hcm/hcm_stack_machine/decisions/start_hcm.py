from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

class StartHcm(AbstractDecisionElement):
    """
    Initializes HCM. 
    """

    def __init__(self, connector, _):
        super(StartHcm, self).__init__(connector)

    def perform(self, connector, reevaluate=False):      
        if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_START_UP)          
        return self.push(CheckIMU)

    def get_reevaluate(self):
        return False
