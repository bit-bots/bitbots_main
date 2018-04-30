"""
WorldModellCapsule
^^^^^^^^^^^^^^^^^^

Provides informations about the world model.

"""
import math

from humanoid_league_msgs.msg import Position2D


class WorldModelCapsule:
    def __init__(self, ):
        self.position = Position2D()
        self.ball_position_u = None
        self.ball_position_v = None

    def get_current_position(self):
        return self.position.pose.x, self.position.pose.y, self.position.pose.theta

    def get_ball_position_xy(self):
        raise NotImplementedError

    def ball_relative_cb(self, msg):
        self.ball_position_u = msg.ball_relative.x
        self.ball_position_v = msg.ball_relative.y

    def get_ball_position_uv(self):
        return self.ball_position_u, self.ball_position_v

    def get_opp_goal_center_uv(self):
        raise NotImplementedError

    def get_own_goal_center_uv(self):
        raise NotImplementedError

    def get_opp_goal_angle(self):
        raise NotImplementedError

    def get_opp_goal_distance(self):
        raise NotImplementedError

    def get_uv_from_xy(self, x, y):
        """ Returns the relativ positions of the robot to this absolute position"""
        current_position = self.get_current_position()
        x2 = x - current_position[0]
        y2 = y - current_position[1]
        theta = -1 * current_position[2]
        u = math.cos(theta) * x2 + math.sin(theta) * y2
        v = math.cos(theta) * y2 - math.sin(theta) * x2
        return u, v

    def get_distance_to_xy(self, x, y):
        """ Returns distance from robot to given position """

        u, v = self.get_uv_from_xy(x, y)
        dist = math.sqrt(u ** 2 + v ** 2)

        return dist

    def get_ballpos(self):
        raise NotImplementedError

    def position_callback(self, pos):
        self.position = pos
