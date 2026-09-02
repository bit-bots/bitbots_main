import math

from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class BallClose(AbstractDecisionElement):
    """
    Determines whether the ball this robot observed itself is in close range to the robot.
    The distance threshold is set in the config file.
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.ball_close_distance = parameters.get("distance", self.blackboard.config["ball_close_distance"])
        self.ball_close_angle = parameters.get("angle", math.pi)

    def get_ball_distance_and_angle(self) -> tuple[float, float]:
        return self.blackboard.world_model.get_ball_distance(), self.blackboard.world_model.get_ball_angle()

    def perform(self, reevaluate=False):
        distance, angle = self.get_ball_distance_and_angle()

        self.publish_debug_data("ball_distance", distance)
        self.publish_debug_data("ball_angle", angle)

        if distance < self.ball_close_distance and abs(angle) < self.ball_close_angle:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class TeamBallClose(BallClose):
    """
    Determines whether the team ball is in close range to the robot.

    This is used to decide if we are close enough to switch from approaching the ball,
    which is done based on the knowledge of the whole team, to playing the ball,
    which requires this robot to observe the ball itself.
    """

    def get_ball_distance_and_angle(self) -> tuple[float, float]:
        return (
            self.blackboard.world_model.get_team_ball_distance(),
            self.blackboard.world_model.get_team_ball_angle(),
        )
