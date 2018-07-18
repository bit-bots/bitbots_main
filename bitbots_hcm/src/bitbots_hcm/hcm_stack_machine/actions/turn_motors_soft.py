import rospy 
import humanoid_league_msgs.msg
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_stack_machine.hcm_connector


class TurnMotorsSoft(AbstractActionElement):
    """
    turn the motors soft
    """

    def __init__(self, connector, _):
        super(TurnMotorsSoft, self).__init__(connector)

    def perform(self, connector, reevaluate=False):        
        #TODO
        return 