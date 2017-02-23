# -*- coding:utf-8 -*-
"""
KickOffKicker
^^^^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:
* 07.01.15: Created (Marc Bestmann)
"""
import random

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class KickOffKicker(AbstractDecisionModule):
    def __init__(self, _):
        super(KickOffKicker, self).__init__()
        self.kicked = False
        self.direction = None

    def perform(self, connector, reevaluate=False):
        if not self.kicked:
            self.direction = -1 if random.random() < 0.5 else 1
            connector.team_data_capsule().publish_kickoff_strategy(self.direction)
            direction_string = "left" if self.direction == -1 else "right"
            say("Kick off to the " + direction_string)
            self.kicked = True
            if self.direction == -1:
                return self.push(KickBall, "SRK")
            else:
                return self.push(KickBall, "SLK")

        return self.interrupt()
