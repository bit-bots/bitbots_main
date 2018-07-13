from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from bitbots_hcm.hcm_state_machine import StayMotorsOff, TurnMotorsOn, TurnMotorsOff, Record

class MotorOffTimer(AbstractDecisionElement):
    """
    Decides on switching servo power
    """

    def __init__(self, connector, _):
        super(MotorOffTimer, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # check if the time is reached
        if connector.hcm.is_motor_off_time():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_MOTOR_OFF)
            # we do an action sequence to turn off the motors and stay in motor off
            self.push(StayMotorsOff)
            return self.push(TurnMotorsOff)
        elif not connector.hcm.is_motor_off_time() and not connector.hcm.are_motors_on():
            # we have to turn the motors on
            return self.push(TurnMotorsOn)
        else:
            # motors are on and we can continue
            return self.push(CheckMotors)

    def get_reevaluate(self):
        return True
