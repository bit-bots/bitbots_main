import rospy 
import humanoid_league_msgs.msg
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_stack_machine.hcm_connector


class StopWalking(AbstractActionElement):
    """
    Stop the walking
    """

    def perform(self, connector, reevaluate=False):        
        connector.stop_walking()
        return 