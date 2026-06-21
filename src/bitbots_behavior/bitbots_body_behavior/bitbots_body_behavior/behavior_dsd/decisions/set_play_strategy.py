from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from game_controller_hsl_interfaces.msg import GameState


class SetPlayStrategy(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        choose a set paly strategy
        :param reevaluate:
            situation
        :return:
        """

        self.situation = parameters.get("situation", None)

        if self.situation == "free_kick":
            #input logic in case of multiple strategies here
            return 1
        elif self.situation == "throw_in":
            return 1
        elif self.situation == "goal_kick":
            return 1
        elif self.situation == "corner_kick":
            return 1
        else:
            #one is default case for all situations
            return 1

    def get_reevaluate(self):
        """
        Game state can change during the game
        """
        return True
