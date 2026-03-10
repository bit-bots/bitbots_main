from typing import Optional

from bitbots_utils.utils import get_parameters_from_other_node
from game_controller_hsl_interfaces.msg import GameState
from std_msgs.msg import Bool

from bitbots_blackboard.capsules import AbstractBlackboardCapsule


class GameStatusCapsule(AbstractBlackboardCapsule):
    """Provides information about the current game state."""

    def __init__(self, node, blackboard=None):
        super().__init__(node, blackboard)
        self.team_id: int = get_parameters_from_other_node(self._node, "parameter_blackboard", ["team_id"])["team_id"]
        self.own_id: int = get_parameters_from_other_node(self._node, "parameter_blackboard", ["bot_id"])["bot_id"]
        self.gamestate = GameState()
        self.last_update: float = 0.0
        self.unpenalized_time: float = 0.0
        self.last_goal_from_us_time = -86400.0
        self.last_goal_time = -86400.0
        self.free_kick_kickoff_team: Optional[bool] = None
        self.game_controller_stop: bool = False
        # publish stopped msg for hcm
        self.stop_pub = node.create_publisher(Bool, "game_controller/stop_msg", 1)

    def get_game_state(self) -> int:
        # Init, ready, set, playing, finished
        return self.gamestate.main_state

    def get_game_phase(self) -> int:
        # Timeout, Normal, Extratime, Penaltyshoot
        return self.gamestate.game_phase

    def get_set_play(self) -> int:
        # None, Direct Freekick, Indirect Freekick, Penalty, Throw in, Goalkick, Cornerkick,
        return self.gamestate.set_play

    def get_secondary_team(self) -> int:
        # Team ID, wer in set Play den Ball hat
        return self.gamestate.kicking_team

    def has_kickoff(self) -> bool:
        # vegelcih mit eigener Teamnummer
        return self.gamestate.kicking_team == self.team_id

    def is_stopped(self) -> bool:
        return self.gamestate.stopped

    def has_penalty_kick(self) -> bool:
        return (
            self.gamestate.set_play == GameState.SET_PLAY_PENALTY_KICK and self.gamestate.kicking_team == self.team_id
        )

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
            self.gamestate.secs_remaining - (self._node.get_clock().now().nanoseconds / 1e9 - self.last_update), 0.0
        )

    def get_secondary_seconds_remaining(self) -> float:
        """Seconds remaining for things like kickoff"""
        # Time from the message minus time passed since receiving it
        return max(
            self.gamestate.secondary_time - (self._node.get_clock().now().nanoseconds / 1e9 - self.last_update),
            0.0,
        )

    def get_seconds_since_unpenalized(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9 - self.unpenalized_time

    def get_is_penalized(self) -> bool:
        return self.gamestate.penalized

    def received_gamestate(self) -> bool:
        return self.last_update != 0.0

    def get_team_id(self) -> int:
        return self.team_id

    def gamestate_callback(self, gamestate_msg: GameState) -> None:
        if self.gamestate.penalized and not gamestate_msg.penalized:
            self.unpenalized_time = self._node.get_clock().now().nanoseconds / 1e9

        if gamestate_msg.own_score > self.gamestate.own_score:
            self.last_goal_from_us_time = self._node.get_clock().now().nanoseconds / 1e9
            self.last_goal_time = self._node.get_clock().now().nanoseconds / 1e9

        if gamestate_msg.rival_score > self.gamestate.rival_score:
            self.last_goal_time = self._node.get_clock().now().nanoseconds / 1e9

        self.game_controller_stop = gamestate_msg.stopped

        self.stop_pub.publish(Bool(data=self.game_controller_stop))

        """Anstoß im Falle von Overtime jetzt erstmal nicht genauer geregelt
        if (
            gamestate_msg.main_state == GameState.STATE_SET
            and self.gamestate.setPlay != 2
            and gamestate_msg.state == GameState.STATE_PLAYING
        ):
            # secondary action is now executed but we will not see this in the new messages.
            # it will look like a normal kick off, but we need to remember that this is some sort of free kick
            # we set the kickoff value accordingly, then we will not be allowed to move if it is a kick for the others
            self.free_kick_kickoff_team = gamestate_msg.kicking_team

        if gamestate_msg.set_play != 2 and gamestate_msg.secondary_time == 0:
            self.free_kick_kickoff_team = gamestate_msg.kicking_team

        if gamestate_msg.set_play != 2 and gamestate_msg.secondary_time == 0:
            self.free_kick_kickoff_team = None

        if self.free_kick_kickoff_team is not None:
            gamestate_msg.has_kick_off = self.free_kick_kickoff_team == self.team_id
        """
        self.last_update = self._node.get_clock().now().nanoseconds / 1e9
        self.gamestate = gamestate_msg
