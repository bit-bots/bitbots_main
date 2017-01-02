# -*- coding:utf-8 -*-
"""
FieldieSearchDecision
^^^^^^^^^^^^^^^^^^^^

This Module decides if the Fieldie Sould Search or Walk/Turn to Find the Ball

The Search is continued untel someone interrupts it by reevaluate or interrupt

History:
''''''''

* 06.12.14 Created (Nils Rokita)

"""
import time
from bitbots.modules.abstract.abstract_action_module import AbstractActionModule
from bitbots.modules.behaviour.body.actions.plain_walk_action import PlainWalkAction
from bitbots.modules.behaviour.body.actions.search import StopAndSearch
from bitbots.modules.behaviour.modell.capsules.walking_capsule import WalkingCapsule
from bitbots.util import get_config


class FieldieSearchDecision(AbstractActionModule):
    def __init__(self, _):
        super(FieldieSearchDecision, self).__init__()
        config = get_config()
        self.start_time = time.time()
        self.turn_wait_time = config["Behaviour"]["Common"]["Search"]["turnWaitTime"]

    def perform(self, connector, reevaluate=False):
        # if we have loked to long in the same direktion without seeing we want to walk
        if time.time() - self.start_time > self.turn_wait_time:
            # reset the timer
            self.start_time = time.time()
            return self.push(PlainWalkAction,
                             [[WalkingCapsule.ZERO, WalkingCapsule.MEDIUM_ANGULAR_RIGHT, WalkingCapsule.ZERO, 5]])
        else:
            # Just Search
            return self.push(StopAndSearch)

            # todo run to the middle point of the field after a long while
