from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode


class SearchFieldFeatures(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(SearchFieldFeatures, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.blackboard.set_head_duty(HeadMode.FIELD_FEATURES)
