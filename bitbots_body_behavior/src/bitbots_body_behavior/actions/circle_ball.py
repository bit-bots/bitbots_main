from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode


class CircleBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(CircleBall, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        # TODO walk around ball but without getting out of good kicking distance
