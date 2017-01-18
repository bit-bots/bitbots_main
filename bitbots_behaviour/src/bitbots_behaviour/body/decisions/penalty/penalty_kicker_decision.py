# -*- coding:utf-8 -*-
"""
PenaltyKickerDecision
^^^^^^^^^^^^^^^^^^^^^

Start of the penalty kicker behaviour.

History:
* 06.12.14: Created (Marc Bestmann)
"""
from stackmachine.abstract_decision_module import AbstractDecisionModule

from body.actions.plain_walk_action import PlainWalkAction
from body.decisions.common.ball_seen import BallSeenPenaltyKick
from stackmachine.model.capsules.walking_capsule import WalkingCapsule


class PenaltyKickerDecision(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):

        if not connector.blackboard_capsule().get_first_steps_done():
            self.do_not_reevaluate()
            connector.blackboard_capsule().set_first_steps_done()
            return self.push(PlainWalkAction,
                             [[WalkingCapsule.FAST_FORWARD, WalkingCapsule.ZERO, WalkingCapsule.ZERO, 6]])
        else:

            return self.push(BallSeenPenaltyKick)
