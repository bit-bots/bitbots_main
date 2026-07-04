import math

from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class RLDribblePossible(AbstractDecisionElement):
    """Decides whether the learned dribble policy can take over.

    The policy walks to the ball on its own, so it only needs the ball to be
    reasonably close and the robot to be roughly aligned with the desired kick
    direction (not facing the opposite way, which the policy handles poorly
    and where turning with the ball is the better behavior).
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        config = self.blackboard.config["rl_dribble"]
        self.ball_distance_threshold = config["ball_distance_threshold"]
        self.align_threshold = config["align_threshold"]

    def perform(self, reevaluate=False):
        ball_distance = self.blackboard.world_model.get_ball_distance()
        ball_near = ball_distance < self.ball_distance_threshold
        self.publish_debug_data(f"Ball distance (needs <{self.ball_distance_threshold})", ball_distance)

        # Alignment between our heading and the map-goal kick direction.
        _, _, direction_rad_map, _ = self.blackboard.pathfinding.get_map_goal(distance=0.0)
        _, _, robot_theta = self.blackboard.world_model.get_current_position()
        align_error = abs(math.atan2(math.sin(direction_rad_map - robot_theta), math.cos(direction_rad_map - robot_theta)))
        aligned = align_error < self.align_threshold
        self.publish_debug_data(f"Kick direction alignment error (needs <{self.align_threshold})", align_error)

        if ball_near and aligned:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
