from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard


class AbstractHCMDecisionElement(AbstractDecisionElement):
    """
    AbstractHCMDecisionElement with a hcm blackboard as its blackboard
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: HcmBlackboard
