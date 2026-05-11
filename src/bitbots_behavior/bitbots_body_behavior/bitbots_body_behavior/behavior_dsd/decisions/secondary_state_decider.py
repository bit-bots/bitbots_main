from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from game_controller_hsl_interfaces.msg import GameState


class SecondaryStateDecider(AbstractDecisionElement):
    """
    Decides in which secondary state the game is currently in. The mode of the secondary state is handled in the
    game controller receiver, so the behavior does ont need to deal with this.
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

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


class SecondaryStateTeamDecider(AbstractDecisionElement):
    """
    Decides if our team or the other team is allowed to execute the secondary state.
    """

    blackboard: BodyBlackboard

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
