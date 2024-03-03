from game_controller_hl_interfaces.msg import GameState

from bitbots_localization_handler.localization_dsd.decisions import AbstractLocalizationDecisionElement


class CheckGameStateReceived(AbstractLocalizationDecisionElement):
    """
    Checks if gamestate from gamecontroller is received.

    """

    def perform(self, reevaluate=False):
        self.clear_debug_data()

        if not self.blackboard.gamestate.received_gamestate():
            if self.blackboard.initialized:
                return "DO_NOTHING"
            else:
                self.blackboard.initialized = True
                return "NO_GAMESTATE_INIT"

        return "GAMESTATE_RECEIVED"

    def get_reevaluate(self):
        return True


class GameStateDecider(AbstractLocalizationDecisionElement):
    def perform(self, reevaluate=False):
        """
        Translates GameState in Blackboard into DSD Answer
        :param reevaluate:
        :return:
        """
        game_state_number = self.blackboard.gamestate.get_gamestate()

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


class SecondaryStateDecider(AbstractLocalizationDecisionElement):
    """
    Decides in which secondary state the game is currently in. The mode of the secondary state is handled in the
    game controller receiver, so the behavior does ont need to deal with this.
    """

    def perform(self, reevaluate=False):
        state_number = self.blackboard.gamestate.get_secondary_state()

        # todo this is a temporary hack to make GUI work
        if state_number == GameState.STATE_NORMAL:
            return "NORMAL"
        elif state_number == GameState.STATE_PENALTYSHOOT:
            return "PENALTYSHOOT"
        elif state_number == GameState.STATE_OVERTIME:
            return "OVERTIME"
        elif state_number == GameState.STATE_TIMEOUT:
            return "TIMEOUT"
        elif state_number == GameState.STATE_DIRECT_FREEKICK:
            return "DIRECT_FREEKICK"
        elif state_number == GameState.STATE_INDIRECT_FREEKICK:
            return "INDIRECT_FREEKICK"
        elif state_number == GameState.STATE_PENALTYKICK:
            return "PENALTYKICK"
        elif state_number == GameState.STATE_CORNER_KICK:
            return "CORNER_KICK"
        elif state_number == GameState.STATE_GOAL_KICK:
            return "GOAL_KICK"
        elif state_number == GameState.STATE_THROW_IN:
            return "THROW_IN"

    def get_reevaluate(self):
        """
        Secondary game state can change during the game
        """
        return True


class SecondaryStateTeamDecider(AbstractLocalizationDecisionElement):
    """
    Decides if our team or the other team is allowed to execute the secondary state.
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd)
        self.team_id = self.blackboard.gamestate.get_team_id()

    def perform(self, reevaluate=False):
        state_number = self.blackboard.gamestate.get_secondary_state()
        # we have to handle penalty shoot differently because the message is strange
        if state_number == GameState.STATE_PENALTYSHOOT:
            if self.blackboard.gamestate.has_kickoff():
                return "OUR"
            return "OTHER"
        else:
            if self.blackboard.gamestate.get_secondary_team() == self.team_id:
                return "OUR"
            return "OTHER"

    def get_reevaluate(self):
        """
        Secondary state Team can change during the game
        """
        return True


class CheckPenalized(AbstractLocalizationDecisionElement):
    def perform(self, reevaluate=False):
        """
        Determines if the robot is penalized by the game controller.
        """
        self.publish_debug_data("Seconds since unpenalized", self.blackboard.gamestate.get_seconds_since_unpenalized())
        if self.blackboard.gamestate.get_is_penalized():
            return "YES"
        elif self.blackboard.gamestate.get_seconds_since_unpenalized() < 1:
            self.publish_debug_data("Reason", "Just unpenalized")
            return "JUST_UNPENALIZED"
        else:
            return "NO"

    def get_reevaluate(self):
        return True


class InitialToReady(AbstractLocalizationDecisionElement):
    """
    Decides if the ready phase was just started coming from initial
    """

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.previous_game_state_number = self.blackboard.gamestate.get_gamestate()

    def perform(self, reevaluate=False):
        previous_game_state_number = self.previous_game_state_number
        game_state_number = self.blackboard.gamestate.get_gamestate()
        self.previous_game_state_number = game_state_number

        self.publish_debug_data("Previous game state", previous_game_state_number)
        self.publish_debug_data("Current game state", game_state_number)

        if previous_game_state_number == GameState.GAMESTATE_INITIAL and game_state_number == GameState.GAMESTATE_READY:
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True
