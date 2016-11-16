# -*- coding: utf-8 -*-
"""
A implementation for an artificial neuronal network

29.11.2014 (Martin Popppinga) adapted from bachelor thesis
"""

import collections

try:
    from __pypy__ import newlist_hint
except ImportError:
    newlist_hint = lambda size: []

ncounter = 1

p_nr = [7, 3, 3]


def sig(x):
    return x / (1 + abs(x))  # ((1.0 / (1.0 + math.e**-x))-0.5)*2


class Neuron(object):
    def __init__(self):
        self.b = 0
        self.output = 0
        self.sigmoid = sig(self.output)
        self.tau = 3.0

    def f(self, input_l):
        """
        The function to compute the output of a neuron
        :param input_l: Liste mit tupeln aus Gewicht und input
        :type input_l: list
        :return: None
        """
        input_sum = 0
        for x, w in input_l:
            input_sum += x * w

        self.output = -self.sigmoid + 1.0 / self.tau * (input_sum + self.b)
        self.sigmoid = sig(self.output)

    def set_output(self, output):
        self.output = output
        self.sigmoid = sig(output)


class Network(object):
    def __init__(self):
        """
        Creates the network
        :return: None
        """
        global ncounter
        self.id = ncounter
        ncounter += 1
        self.version = 0
        self.parent = 0
        self.layers = []
        for i in range(len(p_nr)):
            l = []
            for x in range(p_nr[i]):
                l.append(Neuron())
            self.layers.append(l)

        self.weights = collections.OrderedDict()

    def __str__(self):
        return str(self.id) + "_" + str(self.parent) + "_" + str(self.version)

    def compute(self, in_values):
        """
        Berechnet den Output des Netzes
        :param in_values: list of integers with input values
        :type in_values: list
        :return:
        """

        for nr in range(len(self.layers[0])):
            self.layers[0][nr].set_output(in_values[nr])

        for l in range(1, len(self.layers) - 1):

            for neuron in self.layers[l]:
                f_input = []
                for inp in self.layers[l - 1]:
                    f_input.append((inp.sigmoid, self.weights[(inp, neuron)]))
                for inp in self.layers[l]:
                    f_input.append((inp.sigmoid, self.weights[(inp, neuron)]))

                neuron.f(f_input)

        return_list = []

        for neuron in self.layers[-1]:
            f_input = []
            for inp in self.layers[-2]:
                f_input.append((inp.sigmoid, self.weights[(inp, neuron)]))
            neuron.f(f_input)

            return_list.append(neuron.output)

        return return_list
