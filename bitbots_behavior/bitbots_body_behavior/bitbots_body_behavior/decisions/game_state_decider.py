from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from game_controller_hl_interfaces.msg import GameState


class GameStateDecider(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Translates GameState in Blackboard into DSD Answer
        :param reevaluate:
        :return:
        """

        game_state_number = self.blackboard.gamestate.get_gamestate()
        # todo this is a temporary hack to make GUI work
        if game_state_number == GameState.GAMESTATE_INITIAL:
            return "INITIAL"
        elif game_state_number == GameState.GAMESTATE_READY:
            return "READY"
        elif game_state_number == GameState.GAMESTATE_SET:
            return "SET"
        elif game_state_number == GameState.GAMESTATE_PLAYING:
            return "PLAYING"
        elif game_state_number == GameState.GAMESTATE_FINISHED:
            return "FINISHED"

    def get_reevaluate(self):
        """
        Game state can change during the game
        """
        return True
