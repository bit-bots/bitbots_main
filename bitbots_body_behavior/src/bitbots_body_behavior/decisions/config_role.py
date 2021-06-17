# -*- coding:utf-8 -*-
import rospy
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class ConfigRole(AbstractDecisionElement):
    """
    Decides what kind of behaviour the robot performs
    """
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.role = self.blackboard.blackboard.duty

    def perform(self, reevaluate=False):
        assert self.role in self.blackboard.config['roles'], "No valid role specified"
        return self.role.upper()

    def get_reevaluate(self):
        """
        The role does not change during the game
        """
        return False
