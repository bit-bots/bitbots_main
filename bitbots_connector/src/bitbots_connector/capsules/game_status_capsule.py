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
        self.kick_off_valid_time = rosparam.get_param("Behaviour/Body/Common/kickOffValidTime")
        self.drop_ball_valid_time = rosparam.get_param("Behaviour/Body/Common/dropBallValidTime")
        self.gamestate = GameState()
        self.last_update = 0

    def is_game_state_equals(self, value):
        assert value in [GameState.STATE_NORMAL, GameState.STATE_PENALTYSHOOT, GameState.STATE_OVERTIME,
                         GameState.STATE_TIMEOUT, GameState.STATE_FREEKICK, GameState.STATE_PENALTYKICK]
        return value == self.get_gamestatus()

    def get_gamestatus(self):
        return self.gamestate.gameState

    def has_kickoff(self):
        return self.gamestate.hasKickOff

    def get_own_goals(self):
        return self.gamestate.ownScore

    def get_opp_goals(self):
        return self.gamestate.rivalScore

    def get_seconds_remaining(self):
        # Time from the message minus time passed since receiving it
        return self.gamestate.secondsRemaining - (rospy.get_time() - self.last_update)

    def get_secondary_seconds_remaining(self):
        """Seconds remaining for things like kickoff"""
        # Time from the message minus time passed since receiving it
        return self.gamestate.secondary_seconds_remaining - (rospy.get_time() - self.last_update)

    def get_seconds_since_last_drop_ball(self):
        """Returns the seconds since the last drop in"""
        if self.gamestate.dropInTime == -1:
            return None
        else:
            # Time from the message plus seconds passed since receiving it
            return self.gamestate.dropInTime + (rospy.get_time() - self.last_update)

    def has_penalty_kick(self):
        return self.gamestate.penaltyShot

    def is_allowed_to_move(self):
        return self.gamestate.allowedToMove or rospy.get_time() - self.last_update > 15

    def gamestate_callback(self, gs):
        self.gamestate = gs
        self.last_update = rospy.get_time()
