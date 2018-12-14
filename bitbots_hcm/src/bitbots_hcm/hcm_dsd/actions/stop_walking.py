import rospy 
import humanoid_league_msgs.msg
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_dsd.hcm_blackboard


class StopWalking(AbstractActionElement):
    """
    Stop the walking
    """

    def perform(self, blackboard, reevaluate=False):
        self.do_not_reevaluate()
        if blackboard.is_currently_walking():
            blackboard.stop_walking()
        else:
            return self.pop()        