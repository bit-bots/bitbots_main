import math

from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class KickOrDribble(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.target_distance = parameters.get("map_goal_target_distance", 0.5)
        self.side_offset = parameters.get("side_offset", 0.0)
        self.threshold_front = parameters.get("threshold_front", 10)
        self.threshold_behind = parameters.get("threshold_behind", 10)

    def perform(self, reevaluate=False):
        """
        Determines whether to kick or dribble based on the angle of the map goal
        """
        map_goal = self.blackboard.pathfinding.get_map_goal(self.target_distance, self.side_offset)
        # no other robots too close
        other_robots_close = self.blackboard.costmap.is_other_robot_close(self.threshold_front, self.threshold_behind)
        # actual set play situation
        set_play_state = self.blackboard.gamestate.get_set_play()

        if abs(map_goal[2]) > math.pi / 2 or (
            other_robots_close and set_play_state == 0
        ):  # point away from opponent goal, so we should dribble
            return "DRIBBLE"
        else:
            return "KICK"

    def get_reevaluate(self):
        return True
