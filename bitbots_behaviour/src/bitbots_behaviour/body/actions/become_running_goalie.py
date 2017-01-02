# -*- coding:utf-8 -*-
"""
BecomeRunningGoalie
^^^^^^^^^^^^^^^^^^^

Changes its role to a running goalie.

History:
* 06.12.14: Created (Marc Bestmann)
"""
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.helper import become_one_time_kicker


class BecomeRunningGoalie(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().set_goalie_out_of_goal(True)
        become_one_time_kicker(connector)  # todo why outsourced?
        return self.interrupt()

