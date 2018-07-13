from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement

class Record(AbstractDecisionElement):
    """
    Decides if the robot is currently recording animations
    """

    def __init__(self, connector, _):
        super(Record, self).__init__(connector)

    def perform(self, connector, reevaluate=False):
        # check if the robot is currently recording animations
        if connector.hcm.robot_picked_up():
            if not reevaluate:
                # only publish the new state if we are not reevaluating
                connector.hcm.publish_state(STATE_RECORD)            
            return self.push(StayRecord)
        else:
            # robot is not recording
            return self.push(PickedUp)

    def get_reevaluate(self):
        return True
