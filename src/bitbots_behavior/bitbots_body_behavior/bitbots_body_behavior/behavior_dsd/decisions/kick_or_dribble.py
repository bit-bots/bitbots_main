import math

from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from game_controller_hsl_interfaces.msg import GameState


class KickOrDribble(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        # upfield is towards the opponent goal, downfield is towards our own goal
        self.threshold_upfield = parameters.get("threshold_upfield", 10)
        self.threshold_downfield = parameters.get("threshold_downfield", 10)

    def perform(self, reevaluate=False):
        """
        Determines whether to kick or dribble based on the angle of the map goal
        """
        # Get the map goal, so we can check the angle.
        # distance or side offset are not relevant here, so we can just use 0.0 for both.
        map_goal = self.blackboard.pathfinding.get_map_goal(distance=0.0, side_offset=0.0)

        # Are no other robots too close?
        other_robots_close = self.blackboard.costmap.is_other_robot_close(
            self.threshold_upfield, self.threshold_downfield
        )
        # Get actual set play situation
        set_play_state = self.blackboard.gamestate.get_set_play()

        map_goal_points_away_from_opp_goal = abs(map_goal[2]) > math.pi / 2
        regular_play_with_nearby_robot = other_robots_close and set_play_state == GameState.SET_PLAY_NONE

        if map_goal_points_away_from_opp_goal or regular_play_with_nearby_robot:
            return "DRIBBLE"
        else:
            return "KICK"

    def get_reevaluate(self):
        return True
