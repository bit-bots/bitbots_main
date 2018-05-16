# -*- coding:utf-8 -*-
"""
CenterDecision
^^^^^^^^^^^^^^

Start of the center player behaviour.

History:
* 06.12.14: Created (Marc Bestmann)
"""
from bitbots_body_behaviour.body.decisions.common.go_to_duty_position import GoToDutyPosition
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class CenterDecision(AbstractDecisionModule):  # todo not yet refactored 6.12.14

    def perform(self, connector, reevaluate=False):
        return self.push(GoToDutyPosition)
