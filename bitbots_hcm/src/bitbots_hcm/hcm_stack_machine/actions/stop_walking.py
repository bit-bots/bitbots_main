import rospy 
import humanoid_league_msgs.msg
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_stack_machine.hcm_connector


class StopWalking(AbstractActionElement):
    """
    Stop the walking
    """

    def perform(self, connector, reevaluate=False): 
        self.do_not_reevaluate()
        if connector.is_currently_walking():
            connector.stop_walking()
        else:
            return self.pop()        