from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

class Falling(AbstractDecisionElement):
    """
    Decides if the robot is currently falling and has to act on this
    """

    def __init__(self, connector, _):
        super(Falling, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # check if the robot is currently falling
        if connector.hcm.is_falling_detection_active() and connector.hcm.is_falling():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_FALLING)
            # we play a stand up animation            
            return self.push(PlayFallingAnimation)
        else:
            # robot is not fallen
            return self.push(BehaviorAnimation)

    def get_reevaluate(self):
        return True
