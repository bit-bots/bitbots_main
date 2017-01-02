# -*- coding:utf-8 -*-
"""
BecomeRunningGoalie
^^^^^^^^^^^^^^^^^^^

Changes its role to a fieldie. This is mostly a semantic module for the behaviour graph.

History:
* 06.12.14: Created (Marc Bestmann)
"""
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule


class BecomeTeamPlayer(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().set_goalie_out_of_goal(True)
        connector.set_duty("TeamPlayer")
        return self.interrupt()
