from bitbots_hcm.hcm_dsd.decisions import AbstractHCMDecisionElement


class TeachingMode(AbstractHCMDecisionElement):
    """
    Decides if the robot is currently in teaching mode.
    In the teaching mode the robot can be puppeteered freely.
    To do this we deactivate the torque on all motors.
    If we leave the teaching mode we activate the torque again.
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.last_state_on = False

    def perform(self, reevaluate=False):
        if self.blackboard.teaching_mode_active:
            self.last_state_on = True
            # We activated the teaching mode
            return "TURN_ON"
        elif self.last_state_on:
            self.last_state_on = False
            # We just deactivated the teaching mode and need to clean up
            return "TURN_OFF"
        else:
            # We are not in the teaching mode
            return "OFF"

    def get_reevaluate(self):
        return True
