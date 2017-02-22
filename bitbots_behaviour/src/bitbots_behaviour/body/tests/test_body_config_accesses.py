#!/usr/bin/env python
# #-*- coding:utf-8 -*-
"""
TestBodyConfigAccesses
^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

12.03.14: Created (Marc)

"""
import unittest

from bitbots.modules.behaviour.body.actions.throw import Throw
from bitbots.modules.behaviour.body.actions.go_to_ball import GoToBall
from bitbots.modules.behaviour.body.actions.align_on_ball import AlignOnBall
from bitbots.modules.behaviour.body.decisions.common.ball_seen import AbstractBallSeen
from bitbots.modules.behaviour.body.decisions.goalie.throw_or_raise_arm import ThrowOrRaiseArm
from bitbots.modules.behaviour.modell.connector import BodyConnector


class TestBodyConfigAccesses(unittest.TestCase):
    """
    This Test just initializes objects of the body modules to test the config accesses. The accesses should always be
    in the constructor, so it fails if there is a wrong value name
    """
    @classmethod
    def setUpClass(cls):
        pass

    def setUp(self):
        self.connector = BodyConnector({})
        self.ball_dangerous = ThrowOrRaiseArm(None)
        self.ball_seen = AbstractBallSeen(None)
        self.throw = Throw(None)
        self.align_on_ball = AlignOnBall(None)
        self.go_to_ball = GoToBall(None)

    def test_test(self):
        print("test")

if __name__ == '__main__':
    unittest.main()
