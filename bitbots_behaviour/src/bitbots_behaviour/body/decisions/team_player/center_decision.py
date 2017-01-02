# -*- coding:utf-8 -*-
"""
CenterDecision
^^^^^^^^^^^^^^

Start of the center player behaviour.

History:
* 06.12.14: Created (Marc Bestmann)
"""
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.decisions.common.corridor import CenterCorridor


class CenterDecision(AbstractDecisionModule):  # todo not yet refactored 6.12.14

    def perform(self, connector, reevaluate=False):
        return self.push(CenterCorridor)
