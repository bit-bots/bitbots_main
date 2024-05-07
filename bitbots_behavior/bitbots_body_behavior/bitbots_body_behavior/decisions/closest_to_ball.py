from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class ClosestToBallNoGoalie(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        my_time_to_ball = self.blackboard.team_data.get_own_time_to_ball()
        rank = self.blackboard.team_data.team_rank_to_ball(my_time_to_ball, count_goalies=False, use_time_to_ball=True)
        self.publish_debug_data("time to ball", my_time_to_ball)
        self.publish_debug_data("Rank to ball", rank)
        if rank == 1:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class ClosestToBall(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        my_time_to_ball = self.blackboard.team_data.get_own_time_to_ball()
        rank = self.blackboard.team_data.team_rank_to_ball(my_time_to_ball, count_goalies=True, use_time_to_ball=True)
        self.publish_debug_data("time to ball", my_time_to_ball)
        self.publish_debug_data("Rank to ball", rank)
        if rank == 1:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class RankToBallNoGoalie(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        my_time_to_ball = self.blackboard.team_data.get_own_time_to_ball()
        rank = self.blackboard.team_data.team_rank_to_ball(my_time_to_ball, count_goalies=False, use_time_to_ball=True)
        nummber_of_active_teammates = self.blackboard.team_data.get_number_of_active_teammates()
        self.publish_debug_data("time to ball", my_time_to_ball)
        self.publish_debug_data("Rank to ball", rank)
        if rank == 1:
            return "FIRST"
        # in case we play with three robots we want rank two to defend
        elif rank == 2 and nummber_of_active_teammates == 4:
            return "SECOND"
        elif rank == 3 or (rank == 2 and nummber_of_active_teammates == 3):
            return "THIRD"
        else:
            # emergency fall back if something goes wrong
            self.blackboard.node.get_logger().warning("Rank to ball had some issues")
            return "FIRST"

    def get_reevaluate(self):
        return True
