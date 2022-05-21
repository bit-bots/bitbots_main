import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class LocalizationAvailable(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(LocalizationAvailable, self).__init__(blackboard, dsd, parameters)
        self.use_localization = blackboard.config['use_localization']  # type: bool

    def perform(self, reevaluate=False):
        """
        Determines whether the localization should be used in the current situation.
        :param reevaluate:
        :return:
        """
        if self.use_localization and self.blackboard.world_model.localization_pose_current():
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True


class LocalizationPrecision(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(LocalizationPrecision, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Determines whether the localization should be used in the current situation.
        :param reevaluate:
        :return:
        """
        if self.blackboard.world_model.localization_precision_in_threshold():
            return 'HIGH'
        return 'LOW'

    def get_reevaluate(self):
        return True