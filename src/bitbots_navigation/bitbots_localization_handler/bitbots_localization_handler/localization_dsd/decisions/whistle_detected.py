from game_controller_hsl_interfaces.msg import GameState

from bitbots_localization_handler.localization_dsd.decisions import AbstractLocalizationDecisionElement
from bitbots_localization_handler.localization_dsd.localization_blackboard import LocalizationBlackboard


class WhistleDetected(AbstractLocalizationDecisionElement):
    
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: LocalizationBlackboard
        self.last_timestep_whistle_detected = self.blackboard.last_timestep_whistle_detected


    def perform(self, reevaluate=False):
        """
        Checks if a whistle was detected
        """

        if self.last_timestep_whistle_detected == self.blackboard.last_timestep_whistle_detected:
            return "NOT_DETECTED"
        else:
            return "DETECTED"
        

    def get_reevaluate(self):
        """
        Game state can change during the game
        """
        return True
