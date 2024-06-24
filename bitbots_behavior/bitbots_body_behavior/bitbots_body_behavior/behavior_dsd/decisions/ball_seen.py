from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class BallSeen(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Determines whether the ball was seen recently (as defined in config)
        :param reevaluate:
        :return:
        """
        self.publish_debug_data(
            "Ball lost time", self.blackboard.node.get_clock().now() - self.blackboard.world_model.ball_last_seen()
        )
        if self.blackboard.world_model.ball_has_been_seen():
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
