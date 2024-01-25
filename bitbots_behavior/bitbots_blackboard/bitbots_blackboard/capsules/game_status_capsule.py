"""
GameStatusCapsule
^^^^^^^^^^^^^^^^^

Provides information about the current game state.
"""
from bitbots_utils.utils import get_parameters_from_other_node
from game_controller_hl_interfaces.msg import GameState
from rclpy.node import Node


class GameStatusCapsule:
    def __init__(self, node: Node):
        self.node = node
        self.team_id = get_parameters_from_other_node(self.node, "parameter_blackboard", ["team_id"])["team_id"]
        self.gamestate = GameState()
        self.last_update = 0
        self.unpenalized_time = 0
        self.last_goal_from_us_time = -86400
        self.last_goal_time = -86400
        self.free_kick_kickoff_team = None

    def is_game_state_equals(self, value):
        assert value in [
            GameState.GAMESTATE_PLAYING,
            GameState.GAMESTATE_FINISHED,
            GameState.GAMESTATE_INITIAL,
            GameState.GAMESTATE_READY,
            GameState.GAMESTATE_SET,
        ]
        return value == self.get_gamestate()

    def get_gamestate(self):
        return self.gamestate.game_state

    def get_secondary_state(self):
        return self.gamestate.secondary_state

    def get_secondary_state_mode(self):
        return self.gamestate.secondary_state_mode

    def get_secondary_team(self):
        return self.gamestate.secondary_state_team

    def has_kickoff(self):
        return self.gamestate.has_kick_off

    def has_penalty_kick(self):
        return (
            self.gamestate.secondary_state == GameState.STATE_PENALTYKICK
            or self.gamestate.secondary_state == GameState.STATE_PENALTYSHOOT
        ) and self.gamestate._secondary_state_team == self.team_id

    def get_own_goals(self):
        return self.gamestate.own_score

    def get_opp_goals(self):
        return self.gamestate.rival_score

    def get_seconds_since_own_goal(self):
        return self.node.get_clock().now().nanoseconds / 1e9 - self.last_goal_from_us_time

    def get_seconds_since_any_goal(self):
        return self.node.get_clock().now().nanoseconds / 1e9 - self.last_goal_time

    def get_seconds_remaining(self):
        # Time from the message minus time passed since receiving it
        return max(
            self.gamestate.seconds_remaining - (self.node.get_clock().now().nanoseconds / 1e9 - self.last_update), 0
        )

    def get_secondary_seconds_remaining(self):
        """Seconds remaining for things like kickoff"""
        # Time from the message minus time passed since receiving it
        return max(
            self.gamestate.secondary_seconds_remaining
            - (self.node.get_clock().now().nanoseconds / 1e9 - self.last_update),
            0,
        )

    def get_seconds_since_last_drop_ball(self):
        """Returns the seconds since the last drop in"""
        if self.gamestate.drop_in_time == -1:
            return None
        else:
            # Time from the message plus seconds passed since receiving it
            return self.gamestate.drop_in_time + (self.node.get_clock().now().nanoseconds / 1e9 - self.last_update)

    def get_seconds_since_unpenalized(self):
        return self.node.get_clock().now().nanoseconds / 1e9 - self.unpenalized_time

    def get_is_penalized(self):
        return self.gamestate.penalized

    def received_gamestate(self):
        return self.last_update != 0

    def get_team_id(self):
        return self.team_id

    def get_red_cards(self):
        return self.gamestate.team_mates_with_red_card

    def gamestate_callback(self, gs: GameState):
        if self.gamestate.penalized and not gs.penalized:
            self.unpenalized_time = self.node.get_clock().now().nanoseconds / 1e9

        if gs.own_score > self.gamestate.own_score:
            self.last_goal_from_us_time = self.node.get_clock().now().nanoseconds / 1e9
            self.last_goal_time = self.node.get_clock().now().nanoseconds / 1e9

        if gs.rival_score > self.gamestate.rival_score:
            self.last_goal_time = self.node.get_clock().now().nanoseconds / 1e9

        if (
            gs.secondary_state_mode == 2
            and self.gamestate.secondary_state_mode != 2
            and gs.game_state == GameState.GAMESTATE_PLAYING
        ):
            # secondary action is now executed but we will not see this in the new messages.
            # it will look like a normal kick off, but we need to remember that this is some sort of free kick
            # we set the kickoff value accordingly, then we will not be allowed to move if it is a kick for the others
            self.free_kick_kickoff_team = gs.secondary_state_team

        if gs.secondary_state_mode != 2 and gs.secondary_seconds_remaining == 0:
            self.free_kick_kickoff_team = None

        if self.free_kick_kickoff_team is not None:
            gs.has_kick_off = self.free_kick_kickoff_team == self.team_id

        self.last_update = self.node.get_clock().now().nanoseconds / 1e9
        self.gamestate = gs
