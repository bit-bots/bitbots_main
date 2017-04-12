"""
WorldModellCapsule
^^^^^^^^^^^^^^^^^^

Provides informations about the world model.

"""
import math
from typing import Tuple

from humanoid_league_msgs.msg import Position2D


class WorldModelCapsule:
    def __init__(self, ):
        self.position = Position2D()

    def get_current_position(self)->Tuple[float, float, float]:
        return self.position.pose.x, self.position.pose.y, self.position.pose.theta

    def get_ball_position_xy(self)->Tuple[float, float]:
        raise NotImplementedError

    def get_ball_position_uv(self)->Tuple[float, float]:
        raise NotImplementedError

    def get_opp_goal_center_uv(self)->Tuple[float, float]:
        raise NotImplementedError

    def get_own_goal_center_uv(self)->Tuple[float, float]:
        raise NotImplementedError

    def get_opp_goal_angle(self)->float:
        raise NotImplementedError

    def get_opp_goal_distance(self)->float:
        raise NotImplementedError

    def get_uv_from_xy(self, x, y)->Tuple[float, float]:
        """ Returns the relativ positions of the robot to this absolute position"""
        current_position = self.get_current_position()
        x2 = x - current_position[0]
        y2 = y - current_position[1]
        theta = -1 * current_position[2]
        u = math.cos(theta) * x2 + math.sin(theta) * y2
        v = math.cos(theta) * y2 - math.sin(theta) * x2
        return u, v

    def get_distance_to_xy(self, x, y)->float:
        """ Returns distance from robot to given position """

        u, v = self.get_uv_from_xy(x, y)
        dist = math.sqrt(u ** 2 + v ** 2)

        return dist

    def get_ballpos(self)->Tuple[float, float]:
        raise NotImplementedError

    def position_callback(self, pos: Position2D):
        self.position = pos
