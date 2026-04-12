from game_controller_hsl_interfaces.msg import GameState

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
        game_state_number = self.blackboard.gamestate.get_main_state()
        # todo this is a temporary hack to make GUI work
        if game_state_number == GameState.STATE_INITIAL:
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


class SecondaryStateDecider(AbstractLocalizationDecisionElement):
    """
    Decides in which secondary state the game is currently in. The mode of the secondary state is handled in the
    game controller receiver, so the behavior does ont need to deal with this.
    """

    def perform(self, reevaluate=False):
        set_play_number = self.blackboard.gamestate.get_set_play()
        game_phase_number = self.blackboard.gamestate.get_game_phase()

        # todo this is a temporary hack to make GUI work
        if game_phase_number == GameState.GAME_PHASE_NORMAL and set_play_number == GameState.SET_PLAY_NONE:
            return "NORMAL"
        elif game_phase_number == GameState.GAME_PHASE_PENALTY_SHOOT_OUT and set_play_number == GameState.SET_PLAY_NONE:
            return "PENALTYSHOOT"
        elif game_phase_number == GameState.GAME_PHASE_EXTRA_TIME and set_play_number == GameState.SET_PLAY_NONE:
            return "OVERTIME"
        elif game_phase_number == GameState.GAME_PHASE_TIMEOUT and set_play_number == GameState.SET_PLAY_NONE:
            return "TIMEOUT"
        elif set_play_number == GameState.SET_PLAY_DIRECT_FREE_KICK:
            return "DIRECT_FREEKICK"
        elif set_play_number == GameState.SET_PLAY_INDIRECT_FREE_KICK:
            return "INDIRECT_FREEKICK"
        elif set_play_number == GameState.SET_PLAY_PENALTY_KICK:
            return "PENALTYKICK"
        elif set_play_number == GameState.SET_PLAY_CORNER_KICK:
            return "CORNER_KICK"
        elif set_play_number == GameState.SET_PLAY_GOAL_KICK:
            return "GOAL_KICK"
        elif set_play_number == GameState.SET_PLAY_THROW_IN:
            return "THROW_IN"
        else:
            self.blackboard.node.get_logger().error(
                f"Unknown secondary state with game phase {game_phase_number} and set play {set_play_number}"
            )
            return "UNKNOWN"

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
        super().__init__(blackboard, dsd, parameters)
        self.team_id = self.blackboard.gamestate.get_team_id()

    def perform(self, reevaluate=False):
        game_phase_number = self.blackboard.gamestate.get_game_phase()
        # we have to handle penalty shoot differently because the message is strange
        if game_phase_number == GameState.GAME_PHASE_PENALTY_SHOOT_OUT:
            if self.blackboard.gamestate.has_kick():
                return "OUR"

            return "OTHER"
        else:
            if self.blackboard.gamestate.get_kicking_team() == self.team_id:
                return "OUR"
            # @TODO: handle this better and potentially adapt KickOffTimeUp
            elif (
                self.blackboard.gamestate.get_kicking_team() == 255 or self.blackboard.gamestate.get_kicking_team() == 0
            ):
                return "NONE"

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
        self.previous_game_state_number = self.blackboard.gamestate.get_main_state()

    def perform(self, reevaluate=False):
        previous_game_state_number = self.previous_game_state_number
        game_state_number = self.blackboard.gamestate.get_main_state()
        self.previous_game_state_number = game_state_number

        self.publish_debug_data("Previous game state", previous_game_state_number)
        self.publish_debug_data("Current game state", game_state_number)

        if previous_game_state_number == GameState.STATE_INITIAL and game_state_number == GameState.STATE_READY:
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True
