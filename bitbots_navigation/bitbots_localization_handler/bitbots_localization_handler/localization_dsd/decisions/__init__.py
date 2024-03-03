from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_localization_handler.localization_dsd.localization_blackboard import LocalizationBlackboard


class AbstractLocalizationDecisionElement(AbstractDecisionElement):
    """
    AbstractLocalizationDecisionElement with a localization blackboard as its blackboard
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: LocalizationBlackboard
