# -*- coding:utf-8 -*-
"""
CenterDecision
^^^^^^^^^^^^^^

Start of the center player behaviour.

History:
* 06.12.14: Created (Marc Bestmann)
"""
from body.decisions.common.corridor import CenterCorridor
from stackmachine.abstract_decision_module import AbstractDecisionModule


class CenterDecision(AbstractDecisionModule):  # todo not yet refactored 6.12.14

    def perform(self, connector, reevaluate=False):
        return self.push(CenterCorridor)
