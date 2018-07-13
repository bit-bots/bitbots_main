from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

class Walking(AbstractDecisionElement):
    """
    Decides if the robot is currently walking
    """

    def __init__(self, connector, _):
        super(Walking, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        if connector.hcm.is_currently_walking():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_WALKING)  
            return self.push(StayWalking)
        else:
            return self.push(Controlable)

    def get_reevaluate(self):
        return True
