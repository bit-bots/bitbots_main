import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class Localization(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(Localization, self).__init__(blackboard, dsd)
        self.use_localization = blackboard.config.use_localization  # type: bool

    def perform(self, reevaluate=False):
        return 'YES' if self.use_localization else 'NO'

    def get_reevaluate(self):
        return False