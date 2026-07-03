import math

from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class KickOrDribble(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.target_distance = parameters.get("map_goal_target_distance", 0.5)
        self.side_offset = parameters.get("side_offset", 0.0)

    def perform(self, reevaluate=False):
        """
        Determines whether to kick or dribble based on the angle of the map goal
        """
        map_goal = self.blackboard.pathfinding.get_map_goal(self.target_distance, self.side_offset)
        if abs(map_goal[2]) > math.pi / 2:  # point away from opponent goal, so we should dribble
            return "DRIBBLE"
        else:
            return "KICK"

    def get_reevaluate(self):
        return True
