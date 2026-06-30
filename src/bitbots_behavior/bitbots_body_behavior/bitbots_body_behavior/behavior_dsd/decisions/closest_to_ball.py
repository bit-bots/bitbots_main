from bitbots_blackboard.body_blackboard import BodyBlackboard
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


class RankToBallWithGoalie(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        optimal_positioning = self.blackboard.positioning.get_formation_assignment()
        own_position = optimal_positioning[self.blackboard.gamestate.get_own_id()]
        role = own_position["role"]
        self.publish_debug_data("Role from positioning", role)
        if role == 1:
            return "STRIKER"
        elif role == 2:
            return "GOALIE"
        elif role == 3:
            return "DEFENDER"
        elif role == 4:
            return "SUPPORTER"
        elif role == 5:
            return "DEFENDER"
        elif role == 6:
            return "DEFENDER"
        elif role == 7:
            return "DEFENDER"
        else:
            # emergency fall back if something goes wrong
            self.blackboard.node.get_logger().warning("Rank to ball had some issues. Role" + role)
            return "STRIKER"

    def get_reevaluate(self):
        return True
