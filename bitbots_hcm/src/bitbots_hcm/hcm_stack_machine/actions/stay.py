import rospy 
import humanoid_league_msgs.msg
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_stack_machine.hcm_connector


class AbstractStay(AbstractActionElement):
    """
    Abstract class to create actions which just stay on top of the stack.
    This can be used to stay in a certain state till some precondition changes.
    Implementations can be used to change the name
    """

    def __init__(self, connector, _):
        super(AbstractPlayAnimation, self).__init__(connector)
        self.first_perform = True

    def perform(self, connector, reevaluate=False):        
        # just do nothing
        return 

class StayControlable(AbstractActionElement):
    pass

class StayWalking(AbstractActionElement):
    pass

class StayAnimationRunning(AbstractActionElement):
    pass

class StayPickedUp(AbstractActionElement):
    pass

class StayMotorsOff(AbstractActionElement):
    pass

class StayInPenalty(AbstractActionElement):
    pass
