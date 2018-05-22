"""
GameStatusCapsule
^^^^^^^^^^^^^^^^^

Provides information about the current game state.

"""
import rosparam
import rospy
from humanoid_league_msgs.msg import GameState


class GameStatusCapsule:
    def __init__(self):
        self.team_id = rospy.get_param("teamid", 8)
        self.kick_off_valid_time = rosparam.get_param("Behaviour/Body/Common/kickOffValidTime")
        self.drop_ball_valid_time = rosparam.get_param("Behaviour/Body/Common/dropBallValidTime")
        self.gamestate = GameState()
        self.last_update = 0

    def is_game_state_equals(self, value):
        assert value in [GameState.GAMESTATE_PLAYING, GameState.GAMESTATE_FINISHED, GameState.GAMESTATE_INITAL,
                         GameState.GAMESTATE_READY, GameState.GAMESTATE_SET]
        return value == self.get_gamestatus()

    def get_gamestatus(self):
        return self.gamestate.gameState

    def get_secondary_state(self):
        return self.gamestate.secondaryState

    def has_kickoff(self):
        return self.gamestate.hasKickOff

    def has_penalty_kick(self):
        return (self.gamestate.secondaryState == GameState.STATE_PENALTYKICK or
                self.gamestate.secondaryState == GameState.STATE_PENALTYSHOOT) and \
               self.gamestate.secondrayStateTeam == self.team_id

    def get_own_goals(self):
        return self.gamestate.ownScore

    def get_opp_goals(self):
        return self.gamestate.rivalScore

    def get_seconds_remaining(self):
        # Time from the message minus time passed since receiving it
        return max(self.gamestate.secondsRemaining - (rospy.get_time() - self.last_update), 0)

    def get_secondary_seconds_remaining(self):
        """Seconds remaining for things like kickoff"""
        # Time from the message minus time passed since receiving it
        return max(self.gamestate.secondary_seconds_remaining - (rospy.get_time() - self.last_update), 0)

    def get_seconds_since_last_drop_ball(self):
        """Returns the seconds since the last drop in"""
        if self.gamestate.dropInTime == -1:
            return None
        else:
            # Time from the message plus seconds passed since receiving it
            return self.gamestate.dropInTime + (rospy.get_time() - self.last_update)

    def is_allowed_to_move(self):
        return self.gamestate.allowedToMove or rospy.get_time() - self.last_update > 15

    def gamestate_callback(self, gs):
        self.gamestate = gs
        self.last_update = rospy.get_time()
