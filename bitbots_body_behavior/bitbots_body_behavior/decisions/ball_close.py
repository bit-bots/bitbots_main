import math

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class BallClose(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.ball_close_distance = parameters.get("distance", self.blackboard.config["ball_close_distance"])
        self.ball_close_angle = parameters.get("angle", math.pi)

    def perform(self, reevaluate=False):
        """
        Determines whether the ball is in close range to the robot. The distance threshold is set in the config file.
        :param reevaluate:
        :return:
        """
        self.publish_debug_data("ball_distance", self.blackboard.world_model.get_ball_distance())
        self.publish_debug_data("ball_angle", self.blackboard.world_model.get_ball_angle())

        if (
            self.blackboard.world_model.get_ball_distance() < self.ball_close_distance
            and abs(self.blackboard.world_model.get_ball_angle()) < self.ball_close_angle
        ):
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
