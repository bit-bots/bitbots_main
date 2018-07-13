from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

class CheckMotors(AbstractDecisionElement):
    """
    Checks if we are getting information from the motors.
    Since the HCM is not able to work without motor connection, we will stop if there are no values.
    """

    def __init__(self, connector, _):
        super(CheckMotors, self).__init__(connector)

    def perform(self, connector, reevaluate=False):      
        if not connector.hcm.are_motors_available():
            # tell that we have a hardware problem
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_HARDWARE_PROBLEM)
            # wait for motors to connect
            return self.push(WaitForMotors)
        return self.push(Record)

    def get_reevaluate(self):
        return True
