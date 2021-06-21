# -*- coding:utf-8 -*-
import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class ClosestToBallNoGoalie(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(ClosestToBallNoGoalie, self).__init__(blackboard, dsd, parameters)
        self.use_time_to_ball = parameters.get("use_time_to_ball", False)

    def perform(self, reevaluate=False):
        ball_distance = self.blackboard.world_model.get_ball_distance()
        rank = self.blackboard.team_data.team_rank_to_ball(ball_distance, count_goalies=False,
                                                           use_time_to_ball=self.use_time_to_ball)
        rospy.logerr(f"use_time_to_ball:{self.use_time_to_ball}, rank: {rank}")
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
        self.use_time_to_ball = parameters.get("use_time_to_ball", False)

    def perform(self, reevaluate=False):
        ball_distance = self.blackboard.world_model.get_ball_distance()
        rank = self.blackboard.team_data.team_rank_to_ball(ball_distance, count_goalies=True,
                                                           use_time_to_ball=self.use_time_to_ball)
        rospy.logerr(f"use_time_to_ball:{self.use_time_to_ball}, rank: {rank}")
        self.publish_debug_data(f"ball distance", ball_distance)
        self.publish_debug_data(f"Rank to ball", rank)
        if rank == 1:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class RankToBallNoGoalie(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.use_time_to_ball = parameters.get("use_time_to_ball", False)

    def perform(self, reevaluate=False):
        ball_distance = self.blackboard.world_model.get_ball_distance()
        rank = self.blackboard.team_data.team_rank_to_ball(ball_distance, count_goalies=False,
                                                           use_time_to_ball=self.use_time_to_ball)
        rospy.logerr(f"use_time_to_ball:{self.use_time_to_ball}, rank: {rank}")
        self.publish_debug_data(f"ball distance", ball_distance)
        self.publish_debug_data(f"Rank to ball", rank)
        if rank == 1:
            return "FIRST"
        elif rank == 2:
            return "SECOND"
        elif rank == 3:
            return "THIRD"
        else:
            # emergency fall back if something goes wrong
            rospy.logwarn("Rank to ball had some issues")
            return "FIRST"

    def get_reevaluate(self):
        return True
