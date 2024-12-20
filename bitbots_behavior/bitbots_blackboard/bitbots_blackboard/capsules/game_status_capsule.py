from typing import Optional

from bitbots_utils.utils import get_parameters_from_other_node
from game_controller_hl_interfaces.msg import GameState

from bitbots_blackboard.capsules import AbstractBlackboardCapsule


class GameStatusCapsule(AbstractBlackboardCapsule):
    """Provides information about the current game state."""

    def __init__(self, node, blackboard=None):
        super().__init__(node, blackboard)
        self.team_id: int = get_parameters_from_other_node(self._node, "parameter_blackboard", ["team_id"])["team_id"]
        self.gamestate = GameState()
        self.last_update: float = 0.0
        self.unpenalized_time: float = 0.0
        self.last_goal_from_us_time = -86400
        self.last_goal_time = -86400
        self.free_kick_kickoff_team: Optional[bool] = None

    def get_gamestate(self) -> int:
        return self.gamestate.game_state

    def get_secondary_state(self) -> int:
        return self.gamestate.secondary_state

    def get_secondary_state_mode(self) -> int:
        return self.gamestate.secondary_state_mode

    def get_secondary_team(self) -> int:
        return self.gamestate.secondary_state_team

    def has_kickoff(self) -> bool:
        return self.gamestate.has_kick_off

    def has_penalty_kick(self) -> bool:
        return (
            self.gamestate.secondary_state == GameState.STATE_PENALTYKICK
            or self.gamestate.secondary_state == GameState.STATE_PENALTYSHOOT
        ) and self.gamestate._secondary_state_team == self.team_id

    def get_our_goals(self) -> int:
        return self.gamestate.own_score

    def get_opp_goals(self) -> int:
        return self.gamestate.rival_score

    def get_seconds_since_own_goal(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9 - self.last_goal_from_us_time

    def get_seconds_since_any_goal(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9 - self.last_goal_time

    def get_seconds_remaining(self) -> float:
        # Time from the message minus time passed since receiving it
        return max(
            self.gamestate.seconds_remaining - (self._node.get_clock().now().nanoseconds / 1e9 - self.last_update), 0.0
        )

    def get_secondary_seconds_remaining(self) -> float:
        """Seconds remaining for things like kickoff"""
        # Time from the message minus time passed since receiving it
        return max(
            self.gamestate.secondary_seconds_remaining
            - (self._node.get_clock().now().nanoseconds / 1e9 - self.last_update),
            0.0,
        )

    def get_seconds_since_last_drop_ball(self) -> Optional[float]:
        """Returns the seconds since the last drop in"""
        if self.gamestate.drop_in_time == -1:
            return None
        else:
            # Time from the message plus seconds passed since receiving it
            return self.gamestate.drop_in_time + (self._node.get_clock().now().nanoseconds / 1e9 - self.last_update)

    def get_seconds_since_unpenalized(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9 - self.unpenalized_time

    def get_is_penalized(self) -> bool:
        return self.gamestate.penalized

    def received_gamestate(self) -> bool:
        return self.last_update != 0.0

    def get_team_id(self) -> int:
        return self.team_id

    def get_red_cards(self) -> int:
        return self.gamestate.team_mates_with_red_card

    def gamestate_callback(self, gamestate_msg: GameState) -> None:
        if self.gamestate.penalized and not gamestate_msg.penalized:
            self.unpenalized_time = self._node.get_clock().now().nanoseconds / 1e9

        if gamestate_msg.own_score > self.gamestate.own_score:
            self.last_goal_from_us_time = self._node.get_clock().now().nanoseconds / 1e9
            self.last_goal_time = self._node.get_clock().now().nanoseconds / 1e9

        if gamestate_msg.rival_score > self.gamestate.rival_score:
            self.last_goal_time = self._node.get_clock().now().nanoseconds / 1e9

        if (
            gamestate_msg.secondary_state_mode == 2
            and self.gamestate.secondary_state_mode != 2
            and gamestate_msg.game_state == GameState.GAMESTATE_PLAYING
        ):
            # secondary action is now executed but we will not see this in the new messages.
            # it will look like a normal kick off, but we need to remember that this is some sort of free kick
            # we set the kickoff value accordingly, then we will not be allowed to move if it is a kick for the others
            self.free_kick_kickoff_team = gamestate_msg.secondary_state_team

        if gamestate_msg.secondary_state_mode != 2 and gamestate_msg.secondary_seconds_remaining == 0:
            self.free_kick_kickoff_team = None

        if self.free_kick_kickoff_team is not None:
            gamestate_msg.has_kick_off = self.free_kick_kickoff_team == self.team_id

        self.last_update = self._node.get_clock().now().nanoseconds / 1e9
        self.gamestate = gamestate_msg
