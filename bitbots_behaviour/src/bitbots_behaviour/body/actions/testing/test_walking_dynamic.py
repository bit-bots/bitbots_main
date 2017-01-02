# -*- coding:utf-8 -*-
"""
TestWalkingDynamic
^^^^^^^^^^^^^^^^


Testing the walking using a dynamic pattern
"""
import random
import time

from bitbots.modules.abstract.abstract_action_module import AbstractActionModule
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.go_to_absolute_position import GoToAbsolutePosition
from bitbots.modules.behaviour.body.actions.plain_walk_action import PlainWalkAction
from bitbots.modules.behaviour.modell.capsules.walking_capsule import WalkingCapsule
from bitbots.modules.keys import DATA_VALUE_STATE_READY
from bitbots.util import get_config

config = get_config()


class TestWalkingDynamic(AbstractActionModule):
    def __init__(self, _):
        super(TestWalkingDynamic, self).__init__()
        self.lastmove = [5,0,0]



    def perform(self, connector, reevaluate=False):

        self.lastmove[0] = max(-10, min(10, self.lastmove[0] + random.randint(-4, 4)))
        self.lastmove[1] = max(-10, min(10, self.lastmove[1] + random.randint(-4, 4)))
        self.lastmove[2] = max(-10, min(10, self.lastmove[2] + random.randint(-4, 4)))

        connector.walking_capsule().start_walking_plain(*self.lastmove)



    def get_reevaluate(self):
        return True

