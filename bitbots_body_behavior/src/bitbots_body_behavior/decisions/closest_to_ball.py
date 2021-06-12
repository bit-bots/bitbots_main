# -*- coding:utf-8 -*-
import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class ClosestToBallNoGoalie(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(ClosestToBallNoGoalie, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        ball_distance = self.blackboard.world_model.get_ball_distance()
        rank = self.blackboard.team_data.team_rank_to_ball(ball_distance, count_goalies=False)
        self.publish_debug_data(f"ball distance", ball_distance)
        self.publish_debug_data(f"Rank to ball", rank)
        if rank == 1:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class ClosestToBall(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(ClosestToBall, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        ball_distance = self.blackboard.world_model.get_ball_distance()
        rank = self.blackboard.team_data.team_rank_to_ball(ball_distance, count_goalies=True)
        self.publish_debug_data(f"ball distance", ball_distance)
        self.publish_debug_data(f"Rank to ball", rank)
        if rank == 1:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
