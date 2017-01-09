# -*- coding:utf-8 -*-
"""
GameStatusCapsule
^^^^^^^^^^^^^^^^^

Provides informations about the current game state.

"""
import time

from humanoid_league_msgs.msg import GameState
from keys import DATA_KEY_OWN_KICK_OF, DATA_KEY_DROP_IN_TIME, DATA_KEY_OWN_GOALS, DATA_KEY_ENEMY_GOALS,\
    DATA_KEY_SECONDS_REMAINING, DATA_KEY_SECONDAR_SECONDS_REMAINING, DATA_VALUE_STATE_PLAYING, DATA_VALUE_STATE_SET,\
    DATA_VALUE_STATE_READY, \
    DATA_VALUE_STATE_INITIAL, DATA_VALUE_STATE_FINISHED, DATA_KEY_DROP_BALL_TIME, \
    DATA_KEY_KICK_OFF_TIME, DATA_KEY_GAME_STATUS

import rosparam


class GameStatusCapsule:
    def __init__(self):
        self.kick_off_valid_time = rosparam.get_param("/Beahviour/Common/kickOffValidTime")
        self.drop_ball_valid_time = rosparam.get_param("/Beahviour/Common/dropBallValidTime")
        self.gamestate = GameState()

    def is_game_state_equals(self, value):
        assert value in [DATA_VALUE_STATE_PLAYING, DATA_VALUE_STATE_SET, DATA_VALUE_STATE_READY,
                         DATA_VALUE_STATE_INITIAL, DATA_VALUE_STATE_FINISHED]
        return value == self.get_gamestatus()

    def get_gamestatus(self):
        return self.gamestate.gameState

    def is_kick_off(self):
        return time.time() - self.gamestate.kickoff_sec < self.kick_off_valid_time

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
        return time.time() - self.get_last_drop_in_time()

    def gamestate_callback(self, gs: GameState):
        self.gamestate = gs
