#!/usr/bin/env python
# #-*- coding:utf-8 -*-
"""
TestWalkingCapsule
^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Sheepy <8kessler@informatik.uni-hamburg.de>

16.04.14: Created (Sheepy)

"""
import unittest

from bitbots.modules.behaviour.modell.connector import Connector
from bitbots.modules.behaviour.modell.capsules.walking_capsule import WalkingCapsule
from bitbots.util import get_config


class TestWalkingCapsule(unittest.TestCase):
    def test_integration(self):
        data = {}
        self.connector = Connector(data)
        self.connector.walking_capsule().start_walking(forward_key=WalkingCapsule.MEDIUM_FORWARD,
                                                       sidewards_key=WalkingCapsule.FAST_SIDEWARDS_RIGHT)

        conf = get_config()["Behaviour"]["Common"]["Walking"]
        self.assertEquals(conf[WalkingCapsule.MEDIUM_FORWARD], data["Walking.Forward"])
        self.assertEquals(conf[WalkingCapsule.FAST_SIDEWARDS_RIGHT], data["Walking.Sideward"])
        self.assertEquals(True, data["Walking.Active"])

    def test_walking_not_active_on_only_zero_keys(self):
        data = {}
        self.walking_capsule = WalkingCapsule(data)

        self.walking_capsule.start_walking()

        self.assertTrue("Walking.Active" not in data)

    def test_walking_not_active_on_only_zero_keys_2(self):
        data = {
            "Walking.Forward": 3,
            "Walking.Sideward": 4,
            "Walking.Angular": 2,
            "Walking.Active": True
        }
        self.walking_capsule = WalkingCapsule(data)

        self.walking_capsule.stop_walking()

        self.assertEquals(0, data["Walking.Forward"])
        self.assertEquals(0, data["Walking.Sideward"])
        self.assertEquals(0, data["Walking.Angular"])
        self.assertEquals(False, data["Walking.Active"])


if __name__ == '__main__':
    unittest.main()
