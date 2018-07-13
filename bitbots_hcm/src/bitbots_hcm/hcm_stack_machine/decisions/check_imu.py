from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

class CheckIMU(AbstractDecisionElement):
    """
    Checks if we are getting information from the IMU.
    Since the HCM can not detect falls without it, we will shut everything down if we dont have an imu.
    """

    def __init__(self, connector, _):
        super(CheckIMU, self).__init__(connector)

    def perform(self, connector, reevaluate=False):      
        if not connector.hcm.is_imu_available():
            # tell that we have a hardware problem
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_HARDWARE_PROBLEM)
            # wait for IMU
            return self.push(WaitForIMU)
        return self.push(Penalty)

    def get_reevaluate(self):
        return True
