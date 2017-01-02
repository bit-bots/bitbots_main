# -*- coding:utf-8 -*-
"""
KickOffStrategy
^^^^^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 5/7/14: Created (sheepy)
* 07.01.15: Complete Refactoring (Marc Bestmann)
"""
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.decisions.kick_off.kick_off_kicker import KickOffKicker
from bitbots.modules.behaviour.body.decisions.kick_off.kick_off_supporter import KickOffSupporterSideDecision
from bitbots.util import get_config


class KickOffRoleDecider(AbstractDecisionModule):
    def __init__(self):
        super(KickOffRoleDecider, self).__init__()
        config = get_config()
        self.decision_distance = config["Behaviour"]["Fieldie"]["KickOff"]["roleDecisionDistance"]

    def perform(self, connector, reevaluate=False):
        # decide which role we have in the kick off

        if connector.raw_vision_capsule().get_ball_info("distance") < self.decision_distance:
            return self.push(KickOffKicker)
        else:
            return self.push(KickOffSupporterSideDecision)

