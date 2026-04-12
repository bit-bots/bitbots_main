from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from game_controller_hsl_interfaces.msg import GameState


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

        game_state_number = self.blackboard.gamestate.get_main_state()
        is_stopped = self.blackboard.gamestate.is_stopped()
        # todo this is a temporary hack to make GUI work
        if is_stopped:
            return "STOPPED"
        elif game_state_number == GameState.STATE_INITIAL:
            return "INITIAL"
        elif game_state_number == GameState.STATE_READY:
            return "READY"
        elif game_state_number == GameState.STATE_SET:
            return "SET"
        elif game_state_number == GameState.STATE_PLAYING:
            return "PLAYING"
        elif game_state_number == GameState.STATE_FINISHED:
            return "FINISHED"
        elif game_state_number == GameState.STATE_STANDBY:
            return "STANDBY"
        else:
            # This should never happen, but all cases required string response
            # as we do not get any stack trace otherwise
            self.blackboard.node.get_logger().error(f"Received unknown game state number: {game_state_number}")
            return "UNKNOWN"

    def get_reevaluate(self):
        """
        Game state can change during the game
        """
        return True
