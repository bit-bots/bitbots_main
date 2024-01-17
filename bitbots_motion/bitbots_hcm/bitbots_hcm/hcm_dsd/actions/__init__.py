from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard

from dynamic_stack_decider.abstract_action_element import \
    AbstractActionElement


class AbstractHCMActionElement(AbstractActionElement):
    """
    AbstractHCMActionElement with a hcm blackboard as its blackboard
    """
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: HcmBlackboard
