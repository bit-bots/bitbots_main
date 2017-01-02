# -*- coding:utf-8 -*-
"""
ThrowInEntryPoint
^^^^^^^^^^^^^^^^

.. moduleauthor:: Sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 23.7.14: Created (Sheepy)

Entry point for the ThrowIn Challenge
"""
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.plain_walk_action import PlainWalkAction
from bitbots.modules.behaviour.body.decisions.common.ball_seen import BallSeenThrowIn
from bitbots.modules.behaviour.modell.capsules.walking_capsule import WalkingCapsule


class ThrowInEntryPoint(AbstractDecisionModule):
    def __init__(self, _):
        super(ThrowInEntryPoint, self).__init__()
        self.walked_forward = False

    def perform(self, connector, reevaluate=False):
        if not self.walked_forward:
            self.walked_forward = True
            return self.push(PlainWalkAction, [
                [WalkingCapsule.FAST_FORWARD, WalkingCapsule.SLOW_ANGULAR_LEFT, WalkingCapsule.ZERO, 25],
                [WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_LEFT, WalkingCapsule.ZERO, 5],
                [WalkingCapsule.FAST_FORWARD, WalkingCapsule.SLOW_ANGULAR_LEFT, WalkingCapsule.ZERO, 10]])

        return self.push(BallSeenThrowIn)
