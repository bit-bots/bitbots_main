# -*- coding:utf-8 -*-
"""
PenaltyKickerDecision
^^^^^^^^^^^^^^^^^^^^^

Start of the penalty kicker behaviour.

History:
* 06.12.14: Created (Marc Bestmann)
"""
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from bitbots_body_behaviour.decisions.common.ball_seen import BallSeenPenaltyKick
from bitbots_body_behaviour.actions.go_to import GoToRelativePosition


class PenaltyKickerDecision(AbstractDecisionElement):
    def perform(self, connector, reevaluate=False):

        if not connector.blackboard.get_first_steps_done():
            self.do_not_reevaluate()
            connector.blackboard.set_first_steps_done()
            return self.push(GoToRelativePosition, (0, 0.5, 0))
        else:
            return self.push(BallSeenPenaltyKick)
