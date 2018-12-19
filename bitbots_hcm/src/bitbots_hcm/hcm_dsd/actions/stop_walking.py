import rospy 
import humanoid_league_msgs.msg
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_dsd.hcm_blackboard


class StopWalking(AbstractActionElement):
    """
    Stop the walking
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(StopWalking, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        if self.blackboard.is_currently_walking():
            self.blackboard.stop_walking()
        else:
            return self.pop()        