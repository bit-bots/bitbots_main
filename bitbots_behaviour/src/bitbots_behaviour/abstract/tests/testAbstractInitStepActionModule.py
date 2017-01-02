# -*- coding:utf-8 -*-
"""
TestAbstractInitStepActionModule
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 5/7/14: Created (sheepy)

"""
import unittest
from bitbots.modules.abstract.abstract_init_action_module import AbstractInitActionModule
from bitbots.modules.behaviour.modell.connector import Connector


class DummyChild(AbstractInitActionModule):
    def __init__(self):
        AbstractInitActionModule.__init__(self)
        self.shall_be_one = 0
        self.shall_be_two = 0

    def perform(self, connector, reevaluate=None):
        self.shall_be_two += 1

    def perform_init(self, connector, reevaluate=None):
        self.shall_be_one += 1


class TestAbstractInitStepActionModule(unittest.TestCase):
    def test_correct_call_behaviour(self):
        dc = DummyChild()

        conn = Connector({})

        dc.perform(connector=conn)
        self.assertEqual(1, dc.shall_be_one)
        self.assertEqual(1, dc.shall_be_two)

        dc.perform(connector=conn)
        self.assertEqual(1, dc.shall_be_one)
        self.assertEqual(2, dc.shall_be_two)

        dc.perform(connector=conn)
        self.assertEqual(1, dc.shall_be_one)
        self.assertEqual(3, dc.shall_be_two)
