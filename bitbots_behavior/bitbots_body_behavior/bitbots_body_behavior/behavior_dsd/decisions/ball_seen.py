from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from rclpy.duration import Duration


class BallSeen(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.ball_lost_time = Duration(seconds=self.blackboard.config["ball_lost_time"])

    def perform(self, reevaluate=False):
        """
        Determines whether the ball was seen recently (as defined in config)
        :param reevaluate:
        :return:
        """

        if self.blackboard.world_model.has_been_seen():  # TODO: change to ball_seen of all robots or not?
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
