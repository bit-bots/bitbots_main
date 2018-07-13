from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

class PickedUp(AbstractDecisionElement):
    """
    Decides if the robot is currently picked up
    """

    def __init__(self, connector, _):
        super(PickedUp, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # check if the robot is currently beeing picked up
        if connector.hcm.robot_picked_up():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_PICKED_UP)
            # we do an action sequence to turn the motors to soft and stay in picked up state
            self.push(StayPickedUp)
            return self.push(TurnMotorsSoft)
        else:
            # robot is not picked up
            return self.push(Fallen)

    def get_reevaluate(self):
        return True
