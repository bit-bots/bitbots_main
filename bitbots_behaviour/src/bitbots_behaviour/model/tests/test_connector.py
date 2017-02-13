# -*- coding:utf-8 -*-
"""
TestConnector
^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:

* 14.12.13: Created (Martin Poppinga)
"""
import unittest
from bitbots.modules.behaviour.modell.connector import Connector


class TestConnector(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print("#### Test Connector ####")

    def test_last_seen(self):
        connector = Connector({"BallLastSeen": 1, "GoalYellowLastSeen": 2, "GoalBlueLastSeen": 3})
        self.assertIs(connector.raw_vision_capsule().get_last_seen("Ball"), 1)
        with self.assertRaises(KeyError):
            connector.raw_vision_capsule().get_last_seen("Fliegendes Schwein")
        with self.assertRaises(KeyError):
            connector.raw_vision_capsule().get_last_seen("")

    def test_set_walking(self):
        data = {"Walking.Active": False, "Walking.Forward": 0, "Walking.Angular": 0}
        connector = Connector(data)
        connector.walking_capsule().start_walking_plain(1, 2)
        self.assertIs(data["Walking.Forward"], 1)
        self.assertIs(data["Walking.Angular"], 2)
        self.assertTrue(data["Walking.Active"])

        connector.walking_capsule().stop_walking()
        self.assertIs(data["Walking.Forward"], 0)
        self.assertIs(data["Walking.Angular"], 0)
        self.assertFalse(data["Walking.Active"])
