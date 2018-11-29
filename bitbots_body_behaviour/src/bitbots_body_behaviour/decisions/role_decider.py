# -*- coding:utf-8 -*-
from bitbots_dsd.abstract_decision_element import AbstractDecisionElement


class RoleDecider(AbstractDecisionElement):
    """
    Decides what kind of behaviour the robot performs
    """
    def __init__(self, blackboard, dsd, parameters=None):
        super(RoleDecider, self).__init__(blackboard, dsd)
        self.role = self.blackboard.blackboard.duty

    def perform(self, reevaluate=False):
        assert self.role in self.blackboard.config['roles'], "No valid role specified"
        return self.role

    def get_reevaluate(self):
        """The role does not change during the game"""
        return False
