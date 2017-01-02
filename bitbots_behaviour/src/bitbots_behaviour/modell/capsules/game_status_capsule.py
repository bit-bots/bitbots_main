# -*- coding:utf-8 -*-
"""
GameStatusCapsule
^^^^^^^^^^^^^^^^^

Provides informations about the current game state.

History:
* 05.02.14: Created (sheepy)
* 07.01.15: Refactore (Marc Bestmann)
"""
import time
from bitbots.modules.keys import DATA_KEY_OWN_KICK_OF, DATA_KEY_DROP_IN_TIME, DATA_KEY_OWN_GOALS, DATA_KEY_ENEMY_GOALS,\
    DATA_KEY_SECONDS_REMAINING, DATA_KEY_SECONDAR_SECONDS_REMAINING, DATA_VALUE_STATE_PLAYING, DATA_VALUE_STATE_SET,\
    DATA_VALUE_STATE_READY, \
    DATA_VALUE_STATE_INITIAL, DATA_VALUE_STATE_FINISHED, DATA_KEY_DROP_BALL_TIME, \
    DATA_KEY_KICK_OFF_TIME, DATA_KEY_GAME_STATUS
from bitbots.util import get_config


class GameStatusCapsule(object):
    def __init__(self, data):
        config = get_config()["Behaviour"]["Common"]
        self.kick_off_valid_time = config["kickOffValidTime"]
        self.drop_ball_valid_time = config["dropBallValidTime"]
        self.data = data

    def is_game_state_equals(self, value):
        assert value in [DATA_VALUE_STATE_PLAYING, DATA_VALUE_STATE_SET, DATA_VALUE_STATE_READY,
                         DATA_VALUE_STATE_INITIAL, DATA_VALUE_STATE_FINISHED]
        return value == self.data[DATA_KEY_GAME_STATUS]

    def get_gamestatus(self):
        return self.data[DATA_KEY_GAME_STATUS]

    def is_kick_off(self):
        return time.time() - self.data[DATA_KEY_KICK_OFF_TIME] < self.kick_off_valid_time

    def is_drop_ball(self):
        return time.time() - self.data[DATA_KEY_DROP_BALL_TIME] < self.drop_ball_valid_time

    def delete_kick_off_flag(self):
        self.data[DATA_KEY_KICK_OFF_TIME] = 0

    def delete_drop_ball_flag(self):
        self.data[DATA_KEY_DROP_BALL_TIME] = 0

    def has_kickoff(self):
        return self.data[DATA_KEY_OWN_KICK_OF]

    def get_own_goals(self):
        return self.data[DATA_KEY_OWN_GOALS]

    def get_opp_goals(self):
        return self.data[DATA_KEY_ENEMY_GOALS]

    def get_seconds_remaining(self):
        return self.data[DATA_KEY_SECONDS_REMAINING]

    def get_secondary_seconds_remaining(self):
        return self.data.get(DATA_KEY_SECONDAR_SECONDS_REMAINING, -1)

    def get_last_drop_in_time(self):
        x = self.data.get(DATA_KEY_DROP_IN_TIME, 0)
        return x if x is not None else 0

    def get_seconds_since_last_drop_ball(self):
        return time.time() - self.get_last_drop_in_time()
