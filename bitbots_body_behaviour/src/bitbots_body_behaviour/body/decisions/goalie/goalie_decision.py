# -*- coding:utf-8 -*-
"""
GoalieDecision
^^^^^^^^^^^^^^

Start of goalie Behaviou. Decides ic the goalie has to go back to its own goal.

History:
* 05.12.14: Created (Marc Bestmann & Nils Rokita)
"""
from body.decisions.common.ball_seen import BallSeenGoalie
from body.decisions.common.go_to_duty_position import GoToDutyPosition
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class GoaliePositionDecision(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        if connector.blackboard.get_goalie_out_of_goal():
            return self.push(GoToDutyPosition)

        return self.push(BallSeenGoalie)
