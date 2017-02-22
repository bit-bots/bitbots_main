"""
TeamDataCapsule
^^^^^^^^^^^^^^^
"""
import math

import rospy
from humanoid_league_msgs.msg import Role
from keys import DATA_KEY_GOALIE_BALL_RELATIVE_POSITION, DATA_KEY_BALL_TIME, \
    DATA_KEY_FIELDIE_BALL_TIME_LIST, DATA_KEY_KICKOFF_OFFENSE_SIDE, \
    DATA_KEY_KICKOFF_OFFENSE_SIDE_RECEIVED
from keys.grid_world_keys import DATA_KEY_OWN_POSITION_GRID


class TeamDataCapsule:
    def __init__(self):
        self.role_sender = None  # type: rospy.Publisher
        self.my_data = dict()

    def get_ball_in_own_half(self):
        raise NotImplementedError
        return self.data.get("Team.BallInOwnHalf", False)

    def get_team_goalie_ball_position(self):
        raise NotImplementedError
        return self.data.get(DATA_KEY_GOALIE_BALL_RELATIVE_POSITION, (999999, 0))

    def get_goalie_ball_distance(self):
        raise NotImplementedError
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
        raise NotImplementedError
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

    def set_role(self, role: int):
        """ Set the Team Role - Need to be in data dict for Comm Modules """
        assert role in [Role.ROLE_STRIKER, Role.ROLE_DEFENDER, Role.ROLE_SUPPORTER, Role.ROLE_GOALIE]
        r = Role()
        r.role = role
        self.role_sender.publish(r)
        self.my_data["role"] = role

    def get_role(self):
        return self.my_data.get("role", None)

    def get_own_position_in_grid(self):
        raise NotImplementedError
        return self.data.get(DATA_KEY_OWN_POSITION_GRID, (0, 0))

    def publish_kickoff_strategy(self, strategy):
        raise NotImplementedError
        """ Here needs to be the mitecom setter for kickoff strategy """
        debug_m(3, "Setting Strategy to", strategy)
        self.data[DATA_KEY_KICKOFF_OFFENSE_SIDE] = strategy

    def get_kickoff_strategy(self):
        raise NotImplementedError
        """  Gets you the kick off strategy as a tuple together with the time this information was recived """
        return self.data.get(DATA_KEY_KICKOFF_OFFENSE_SIDE_RECEIVED, 0)
