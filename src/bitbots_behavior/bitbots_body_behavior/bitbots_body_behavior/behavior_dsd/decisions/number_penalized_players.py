from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from game_controller_hsl_interfaces.msg import GameState


class NumberPenalizedTeamMates(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Return number of penalized team mates
        :param reevaluate:
        :return:
        """
        game_state_penalized_team_mates = self.blackboard.gamestate.get_penalized_team_mates()

        if game_state_penalized_team_mates == 4:
            return "FOUR"
        elif game_state_penalized_team_mates == 3:
            return "THREE"
        elif game_state_penalized_team_mates == 2:
            return "TWO"
        elif game_state_penalized_team_mates == 1:
            return "ONE"
        else:
            return "ZERO"

    def get_reevaluate(self):
        """
        Game state can change during the game
        """
        return True
    
class NumberPenalizedRivals(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Return number of penalized rivals
        :param reevaluate:
        :return:
        """
        game_state_penalized_rivals = self.blackboard.gamestate.get_penalized_rivals()

        if game_state_penalized_rivals == 4:
            return "FOUR"
        elif game_state_penalized_rivals == 3:
            return "THREE"
        elif game_state_penalized_rivals == 2:
            return "TWO"
        elif game_state_penalized_rivals == 1:
            return "ONE"
        else:
            return "ZERO"

    def get_reevaluate(self):
        """
        Game state can change during the game
        """
        return True
