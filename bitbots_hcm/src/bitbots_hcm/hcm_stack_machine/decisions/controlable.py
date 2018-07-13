from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

class Controlable(AbstractDecisionElement):
    """
    Decides if the robot is currently walking
    """

    def __init__(self, connector, _):
        super(Controlable, self).__init__(connector)

    def perform(self, connector, reevaluate=False):        
        if not reevaluate:
            # only publish the new state if we are not reevaluating
            connector.hcm.publish_state(STATE_CONTROLABLE)  
        return self.push(StayControlable)

    def get_reevaluate(self):
        return True
