# -*- coding:utf-8 -*-
"""
KickOffStrategy
^^^^^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 5/7/14: Created (sheepy)
* 07.01.15: Complete Refactoring (Marc Bestmann)
"""
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from bitbots_body_behaviour.body.decisions.kick_off.kick_off_kicker import KickOffKicker
from bitbots_body_behaviour.body.decisions.kick_off.kick_off_supporter import KickOffSupporterSideDecision


class KickOffRoleDecider(AbstractDecisionElement):
    def __init__(self, connector):
        super(KickOffRoleDecider, self).__init__(connector)
        self.decision_distance = connector.config["Body"]["Fieldie"]["KickOff"]["roleDecisionDistance"]

    def perform(self, connector, reevaluate=False):
        # decide which role we have in the kick off

        if connector.world_model.get_ball_distance() < self.decision_distance:
            return self.push(KickOffKicker)
        else:
            return self.push(KickOffSupporterSideDecision)

