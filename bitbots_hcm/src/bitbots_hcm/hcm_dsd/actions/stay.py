import rospy
import humanoid_league_msgs.msg
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard, STATE_HCM_OFF, STATE_PENALTY


class AbstractStay(AbstractActionElement):
    """
    Abstract class to create actions which just stay on top of the stack.
    This can be used to stay in a certain state till some precondition changes.
    Implementations can be used to change the name
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractStay, self).__init__(blackboard, dsd, parameters)

    def perform(self):
        # just do nothing
        return


class StayControlable(AbstractStay):
    pass


class StayWalking(AbstractStay):
    pass


class StayAnimationRunning(AbstractStay):
    pass


class StayPickedUp(AbstractStay):
    pass


class StayMotorsOff(AbstractStay):
    pass


class StayInPenalty(AbstractStay):
    def perform(self):
        self.blackboard.current_state = STATE_PENALTY


class StayRecord(AbstractStay):
    pass


class StayShutDown(AbstractStay):
    def perform(self):
        self.blackboard.current_state = STATE_HCM_OFF
