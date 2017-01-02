# -*- coding:utf-8 -*-
"""
TeamDataCapsule
^^^^^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 5/4/14: Created (sheepy)

"""
import math

from bitbots.debug import Scope
from bitbots.modules.abstract.abstract_module import debug_m
from bitbots.modules.keys import DATA_KEY_GOALIE_BALL_RELATIVE_POSITION, DATA_KEY_BALL_TIME, \
    DATA_KEY_FIELDIE_BALL_TIME_LIST, DATA_KEY_ROLE, DATA_KEY_KICKOFF_OFFENSE_SIDE, \
    DATA_KEY_KICKOFF_OFFENSE_SIDE_RECEIVED
from bitbots.modules.keys.grid_world_keys import DATA_KEY_OWN_POSITION_GRID
from bitbots.util import get_config
from mitecom.mitecom import ROLE_STRIKER, ROLE_DEFENDER, ROLE_SUPPORTER, ROLE_GOALIE


config = get_config()


class TeamDataCapsule:
    def __init__(self, data):
        self.data = data
        self.debug = Scope("Connector.Capsule.TeamDataCapsule")

        self.my_player_number = config["PLAYER"]

    def get_ball_in_own_half(self):
        return self.data.get("Team.BallInOwnHalf", False)

    def get_team_goalie_ball_position(self):
        return self.data.get(DATA_KEY_GOALIE_BALL_RELATIVE_POSITION, (999999, 0))

    def get_goalie_ball_distance(self):
        try:
            distance = math.sqrt(self.data[DATA_KEY_GOALIE_BALL_RELATIVE_POSITION][0] ** 2 +
                                 self.data[DATA_KEY_GOALIE_BALL_RELATIVE_POSITION][1] ** 2)
        except KeyError:
            distance = 0
        return distance

    def team_rank_to_ball(self):
        """
        Returns the position of the current robot of the distance to the ball
        """
        ball_time = 0 if self.data[DATA_KEY_BALL_TIME] == 99999999 else \
            self.data[DATA_KEY_BALL_TIME]
        plist = self.data.get(DATA_KEY_FIELDIE_BALL_TIME_LIST, [])
        plist.append((self.my_player_number, ball_time))
        plist = sorted(plist, key=lambda xl: xl[1])
        # self.debug(str(plist))
        try:
            position = \
                [i for i, x in enumerate(plist) if x[0] == config["PLAYER"]][0] + 1
        except IndexError:
            debug_m(1, "I do not exist :(")
            position = 1

        return position

    def set_role(self, role):
        """ Set the Team Role - Need to be in data dict for Comm Modules """
        assert role in [ROLE_STRIKER, ROLE_DEFENDER, ROLE_SUPPORTER, ROLE_GOALIE]
        self.data[DATA_KEY_ROLE] = role

    def get_role(self):
        return self.data.get(DATA_KEY_ROLE, None)

    def get_own_position_in_grid(self):
        return self.data.get(DATA_KEY_OWN_POSITION_GRID, (0, 0))

    def publish_kickoff_strategy(self, strategy):
        """ Here needs to be the mitecom setter for kickoff strategy """
        debug_m(3, "Setting Strategy to", strategy)
        self.data[DATA_KEY_KICKOFF_OFFENSE_SIDE] = strategy

    def get_kickoff_strategy(self):
        """  Gets you the kick off strategy as a tuple together with the time this information was recived """
        return self.data.get(DATA_KEY_KICKOFF_OFFENSE_SIDE_RECEIVED, 0)
