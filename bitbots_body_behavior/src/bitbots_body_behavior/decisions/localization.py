import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class Localization(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(Localization, self).__init__(blackboard, dsd, parameters)
        self.use_localization = blackboard.config['use_localization']  # type: bool

    def perform(self, reevaluate=False):
        """
        Determines whether the localization should be used in the current situation.
        :param reevaluate:
        :return:
        """
        if self.use_localization and self.blackboard.world_model.localization_precision_in_threshold():
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
