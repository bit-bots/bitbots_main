from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class AvoidBall(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Determines whether we currently avoid the ball or not
        :param reevaluate:
        :return:
        """
        self.publish_debug_data("avoid_ball", self.blackboard.pathfinding.avoid_ball)

        if self.blackboard.pathfinding.avoid_ball:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
