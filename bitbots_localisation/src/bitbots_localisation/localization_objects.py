# -*- coding:utf-8 -*-
import math
import random

import itertools
import timeit

import numpy as np
import time


class Particle:
    """
    Particle which determine possible robot localisation
    """
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
        self.weight = 0

    def rotate(self, r):
        self.r += r

    def move_forward(self, dist):
        self.x += dist * math.cos(math.radians(self.r))
        self.y += dist * math.sin(math.radians(self.r))

    @staticmethod
    def create_n_random(nr, lbx, lby, ubx, uby):
        ret = []
        for x in range(nr):
            x = random.randint(lbx, ubx)
            y = random.randint(lby, uby)
            r = random.randint(0, 360)
            ret.append(Particle(x, y, r))
        return ret


class Field:
    """
    Field description
    """
    def __init__(self):
        # todo from config!
        self.x_size = 10000
        self.y_size = 6000

        # static
        self.goalposts = ((5000, 1200), (5000, -1200), (-5000, 1200), (-5000, -1200))

        self.lines = [
            ((0, 0), (0, 0)),
            ((0, 0), (0, 0)),
            ((0, 0), (0, 0)),
            ((0, 0), (0, 0)),
            ((0, 0), (0, 0)),
            ((0, 0), (0, 0)),
            ((0, 0), (0, 0))
        ]

        self.cicle = []

        self.points = [(0, 0)]

        # todo add dynamic
        # todo add optional
        # -> SLAM

    def get_dimensions(self):
        """
        Returns the size of the current field
        :return:
        :rtype tuple(int, int, int, int)
        """
        return -(self.x_size / 2), -(self.y_size / 2),  (self.x_size / 2), (self.y_size / 2)

    def mes_goals(self, prtkl):
        """
        Get all goal distances for one particle
        :param prtkl: list with particle
        :return:
        """

        ret = ([0, 0], [0, 0], [0, 0], [0, 0])
        # u,v, u,v, u,v, u,v
        gp = self.goalposts
        for p in range(4):
            rad = math.radians(prtkl[2])

            #ret[p], ret[p+1] = self.uv(prtkl[0], prtkl[1], prtkl[2], gp[p], gp[p+1])
            ret[p][0] = -(prtkl[0] - gp[p][0]) * math.cos(rad) + (prtkl[1] - gp[p][1]) * math.sin(-rad)
            ret[p][1] = (prtkl[0] - gp[p][0]) * math.sin(rad) - (prtkl[1] - gp[p][1]) * math.cos(-rad)

        return ret

    @staticmethod
    def uv(x, y, ang, xo, yo):
        """
        :param x: x position of the robot
        :param y: y position of the robot
        :param ang: orientation of the robot in degrees
        :param xo: x position of the object
        :param yo: y position of the object
        computes the u and v values based on x,y values
        """
        rad_ang = math.radians(ang)
        u = -(x - xo) * math.cos(rad_ang) + (y - yo) * math.sin(-rad_ang)
        v = (x - xo) * math.sin(rad_ang) - (y - yo) * math.cos(rad_ang)

        return u, v
