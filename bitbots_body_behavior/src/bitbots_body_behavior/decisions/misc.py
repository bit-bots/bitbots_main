# -*- coding:utf-8 -*-
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class Yes(AbstractDecisionElement):
    """
    Decides 'YES'
    """
    def __init__(self, blackboard, dsd, parameters=None):
        super(Yes, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        return "YES"

    def get_reevaluate(self):
        """
        The role does not change during the game
        """
        return False
