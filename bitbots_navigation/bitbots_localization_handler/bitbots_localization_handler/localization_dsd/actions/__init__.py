# Setting up runtime type checking for this package
# We need to do this again here because the dsd imports
# the decisions and actions from this package in a standalone way
from beartype.claw import beartype_this_package
from dynamic_stack_decider.abstract_action_element import AbstractActionElement

from bitbots_localization_handler.localization_dsd.localization_blackboard import LocalizationBlackboard

beartype_this_package()


class AbstractLocalizationActionElement(AbstractActionElement):
    """
    AbstractLocalizationActionElement with a localization blackboard as its blackboard
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: LocalizationBlackboard
