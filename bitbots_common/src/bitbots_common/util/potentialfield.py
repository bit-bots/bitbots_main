# -*- coding: utf-8 -*-
"""
An implementation for apotential filed

1.12.2014 (Martin Popppinga) adapted from bachelor thesis
"""

import math


class PotentialField(object):
    def __init__(self, attract):
        self.vector = [0, 0]
        self.attract = attract
        self.p_activated_attractors = True  # TODO config

    def update(self, ob):
        obx = ob[0]
        oby = ob[1]
        if obx != 0 or oby != 0:
            dist = math.sqrt(obx ** 2 + oby ** 2)
            obx /= dist
            oby /= dist
            force = 500.0 / dist
            if not self.attract:
                force **= 2

                self.vector[0] = -obx * force
                self.vector[1] = -oby * force * 2.0
            else:
                # constant attractor
                self.vector[0] = -obx
                self.vector[1] = -oby

            if self.attract:
                if self.p_activated_attractors:
                    self.vector[0] *= -5.0
                    self.vector[1] *= -5.0
                else:
                    self.vector = [0, 0]


class PotentialMap(object):
    """
    Definiert die Potential Fields
    """

    def __init__(self, (nr_r, nr_a)):
        """
        :param nr_r: number of obstacles to build a potential field
        """
        self.p_activated_attractors = False  # TODO config
        self.p_ball_repulsor = True  # todo config

        self.fields = []
        for i in range(nr_r):
            self.fields.append(PotentialField(False))

        for i2 in range(nr_a):
            self.fields.append(PotentialField(True))

    def compute(self, oblist):
        """
        :return the 3-Tupel vektor
        """
        if len(oblist) == 0:
            return 0, 0, 0
        
        x = 0
        for field in self.fields:
            field.update(oblist[x])
            x += 1

        vectorx, vectory = (0, 0)
        x = 0
        for field in self.fields:
            if (self.p_ball_repulsor and not self.p_activated_attractors and x == (len(oblist)-1)) or \
                    (self.p_ball_repulsor and self.p_activated_attractors and x == (len(oblist)-2)):
                #its a ball repulsor, reduced force
                vectorx += field.vector[0] / 3.0
                vectory += field.vector[1] / 3.0

            else:
                vectorx += field.vector[0]
                vectory += field.vector[1]
            x += 1

        vectory /= float(len(self.fields))
        vectorx /= float(len(self.fields))

        forw = vectorx
        turn = 0  # todo sollte auch berchnet werden
        side = vectory

        return forw, turn, side
