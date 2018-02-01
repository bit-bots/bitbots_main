"""
GameStatusCapsule
^^^^^^^^^^^^^^^^^

Provides informations about the current game state.

"""
import time

import rosparam
from bitbots_body_behaviour.keys import DATA_VALUE_STATE_READY, DATA_VALUE_STATE_PLAYING, \
    DATA_VALUE_STATE_SET, DATA_VALUE_STATE_FINISHED, DATA_VALUE_STATE_INITIAL
from humanoid_league_msgs.msg import GameState
import rospy

class GameStatusCapsule:
    def __init__(self):
        self.kick_off_valid_time = rosparam.get_param("Behaviour/Body/Common/kickOffValidTime")
        self.drop_ball_valid_time = rosparam.get_param("Behaviour/Body/Common/dropBallValidTime")
        self.gamestate = GameState()
        self.lastupdate = 0

    def is_game_state_equals(self, value):
        assert value in [DATA_VALUE_STATE_PLAYING, DATA_VALUE_STATE_SET, DATA_VALUE_STATE_READY,
                         DATA_VALUE_STATE_INITIAL, DATA_VALUE_STATE_FINISHED]
        return value == self.get_gamestatus()

    def get_gamestatus(self):
        return self.gamestate.gameState

    def is_kick_off(self):
        return rospy.get_time() - self.gamestate.kickoff_sec < self.kick_off_valid_time

    def is_drop_ball(self):
        raise NotImplemented

    def delete_kick_off_flag(self):
        raise NotImplemented
        self.data[DATA_KEY_KICK_OFF_TIME] = 0

    def delete_drop_ball_flag(self):
        raise NotImplemented
        self.data[DATA_KEY_DROP_BALL_TIME] = 0

    def has_kickoff(self):
        return self.gamestate.hasKickOff

    def get_own_goals(self):
        return self.gamestate.ownScore

    def get_opp_goals(self):
        return self.gamestate.rivalScore

    def get_seconds_remaining(self):
        raise NotImplemented

    def get_secondary_seconds_remaining(self):
        raise NotImplemented
        return self.data.get(DATA_KEY_SECONDAR_SECONDS_REMAINING, -1)

    def get_last_drop_in_time(self):
        raise NotImplemented
        x = self.data.get(DATA_KEY_DROP_IN_TIME, 0)
        return x if x is not None else 0

    def get_seconds_since_last_drop_ball(self):
        return rospy.get_time() - self.get_last_drop_in_time()

    def has_penalty_kick(self):
        return self.gamestate.penaltyShot

    def is_allowed_to_move(self):
        return self.gamestate.allowedToMove or rospy.get_time() - self.lastupdate > 15

    def gamestate_callback(self, gs: GameState):
        self.gamestate = gs
        self.lastupdate = rospy.get_time()
