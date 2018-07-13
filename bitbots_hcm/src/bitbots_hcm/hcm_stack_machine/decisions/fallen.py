from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

class Fallen(AbstractDecisionElement):
    """
    Decides if the robot is currently fallen and lying on the ground
    """

    def __init__(self, connector, _):
        super(Fallen, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # check if the robot is currently laying on the ground
        if connector.hcm.is_stand_up_active() and connector.hcm.is_fallen():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_FALLEN)
            # we play a stand up animation            
            return self.push(PlayAnimationStandUp)
        else:
            # robot is not fallen
            return self.push(Falling)

    def get_reevaluate(self):
        return True
