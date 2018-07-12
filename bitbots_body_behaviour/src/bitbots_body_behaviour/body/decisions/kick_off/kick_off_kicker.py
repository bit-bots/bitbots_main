# -*- coding:utf-8 -*-
"""
KickOffKicker
^^^^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:
* 07.01.15: Created (Marc Bestmann)
"""
import random

from bitbots_body_behaviour.body.actions.kick_ball import KickBall
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from humanoid_league_msgs.msg import Strategy


class KickOffKicker(AbstractDecisionElement):
    def __init__(self, connector):
        super(KickOffKicker, self).__init__(connector)
        self.kicked = False
        self.direction = None

    def perform(self, connector, reevaluate=False):
        if not self.kicked:
            self.direction = Strategy.SIDE_LEFT if random.random() < 0.5 else Strategy.SIDE_RIGHT
            connector.team_data.publish_kickoff_strategy(self.direction)
            self.kicked = True
            if self.direction == Strategy.SIDE_RIGHT:
                return self.push(KickBall, "RIGHT_SIDE_KICK")
            else:
                return self.push(KickBall, "LEFT_SIDE_KICK")

        return self.interrupt()
