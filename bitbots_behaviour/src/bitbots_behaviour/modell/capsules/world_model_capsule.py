# -*- coding:utf-8 -*-
"""
WorldModellCapsule
^^^^^^^^^^^^^^^^^^

Provides informations about the world model.

History:
* 05.12.14: Created (Marc Bestmann)

"""
import math

from bitbots.modules.keys import DATA_KEY_GOAL_MODEL


class WorldModelCapsule(object):
    def __init__(self, data):
        self.data = data

    def get_current_position(self):
        return self.data.get(DATA_KEY_GOAL_MODEL, None).get_robot_absolute_position()

    def get_ball_position_xy(self):
        raise NotImplementedError

    def get_ball_position_uv(self):
        return self.get_uv_from_xy(*self.get_ball_position_xy())

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
