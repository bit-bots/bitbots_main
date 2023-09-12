from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import RobotControlState


class AbstractStay(AbstractActionElement):
    """
    Abstract class to create actions which just stay on top of the stack.
    This can be used to stay in a certain state till some precondition changes.
    Implementations can be used to change the name
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: HcmBlackboard

    def perform(self):
        # just do nothing
        return


class StayControlable(AbstractStay):
    def perform(self):
        self.blackboard.current_state = RobotControlState.CONTROLLABLE


class StayWalking(AbstractStay):
    pass


class StayAnimationRunning(AbstractStay):
    pass


class StayPickedUp(AbstractStay):
    pass


class StayMotorsOff(AbstractStay):
    pass


class StayStopped(AbstractStay):
    def perform(self):
        self.blackboard.current_state = RobotControlState.PENALTY


class StayRecord(AbstractStay):
    pass


class StayKicking(AbstractStay):
    pass
