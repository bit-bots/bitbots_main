"""
GameStatusCapsule
^^^^^^^^^^^^^^^^^

Provides information about the current game state.

"""
import rosparam
import rclpy
from rclpy.node import Node
from humanoid_league_msgs.msg import GameState


class GameStatusCapsule:
    def __init__(self, node: Node):
        self.team_id = self.node.get_parameter('team_id').get_parameter_value().double_value
        self.gamestate = GameState()
        self.last_update = 0
        self.unpenalized_time = 0
        self.last_goal_from_us_time = -86400
        self.last_goal_time = -86400
        self.free_kick_kickoff_team = None

    def is_game_state_equals(self, value):
        assert value in [GameState.GAMESTATE_PLAYING, GameState.GAMESTATE_FINISHED, GameState.GAMESTATE_INITAL,
                         GameState.GAMESTATE_READY, GameState.GAMESTATE_SET]
        return value == self.get_gamestate()

    def get_gamestate(self):
        return self.gamestate.gameState

    def get_secondary_state(self):
        return self.gamestate.secondaryState

    def get_secondary_state_mode(self):
        return self.gamestate.secondaryStateMode

    def get_secondary_team(self):
        return self.gamestate.secondaryStateTeam

    def has_kickoff(self):
        return self.gamestate.hasKickOff

    def has_penalty_kick(self):
        return (self.gamestate.secondaryState == GameState.STATE_PENALTYKICK or
                self.gamestate.secondaryState == GameState.STATE_PENALTYSHOOT) and \
               self.gamestate.secondaryStateTeam == self.team_id

    def get_own_goals(self):
        return self.gamestate.ownScore

    def get_opp_goals(self):
        return self.gamestate.rivalScore

    def get_seconds_since_own_goal(self):
        return float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9) - self.last_goal_from_us_time

    def get_seconds_since_any_goal(self):
        return float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9) - self.last_goal_time

    def get_seconds_remaining(self):
        # Time from the message minus time passed since receiving it
        return max(self.gamestate.secondsRemaining - (float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9) - self.last_update), 0)

    def get_secondary_seconds_remaining(self):
        """Seconds remaining for things like kickoff"""
        # Time from the message minus time passed since receiving it
        return max(self.gamestate.secondary_seconds_remaining - (float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9) - self.last_update), 0)

    def get_seconds_since_last_drop_ball(self):
        """Returns the seconds since the last drop in"""
        if self.gamestate.dropInTime == -1:
            return None
        else:
            # Time from the message plus seconds passed since receiving it
            return self.gamestate.dropInTime + (float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9) - self.last_update)

    def get_seconds_since_unpenalized(self):
        return float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9) - self.unpenalized_time

    def get_is_penalized(self):
        return self.gamestate.penalized

    def received_gamestate(self):
        return self.last_update != 0

    def get_team_id(self):
        return self.team_id

    def get_red_cards(self):
        return self.gamestate.teamMatesWithRedCard

    def gamestate_callback(self, gs):
        if self.gamestate.penalized and not gs.penalized:
            self.unpenalized_time = float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9)

        if gs.ownScore > self.gamestate.ownScore:
            self.last_goal_from_us_time = float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9)
            self.last_goal_time = float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9)

        if gs.rivalScore > self.gamestate.rivalScore:
            self.last_goal_time = float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9)

        if gs.secondaryStateMode == 2 and self.gamestate.secondaryStateMode != 2 \
                and gs.gameState == GameState.GAMESTATE_PLAYING:
            # secondary action is now executed but we will not see this in the new messages.
            # it will look like a normal kick off, but we need to remember that this is some sort of free kick
            # we set the kickoff value accordingly, then we will not be allowed to move if it is a kick for the others
            self.free_kick_kickoff_team = gs.secondaryStateTeam

        if gs.secondaryStateMode != 2 and gs.secondary_seconds_remaining == 0:
            self.free_kick_kickoff_team = None

        if self.free_kick_kickoff_team is not None:
            gs.hasKickOff = self.free_kick_kickoff_team == self.team_id

        self.last_update = float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9)
        self.gamestate = gs
