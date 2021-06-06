# -*- coding:utf-8 -*-
import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class ClosestToBallNoGoalie(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(ClosestToBallNoGoalie, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if self.blackboard.team_data.team_rank_to_ball(count_goalies=False) == 1:
            return "YES"
        return "NO"


class ClosestToBall(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(ClosestToBall, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if self.blackboard.team_data.team_rank_to_ball(count_goalies=True) == 1:
            return "YES"
        return "NO"
