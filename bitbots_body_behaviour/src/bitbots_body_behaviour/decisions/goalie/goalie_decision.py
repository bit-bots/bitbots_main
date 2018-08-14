# -*- coding:utf-8 -*-
"""
GoalieDecision
^^^^^^^^^^^^^^

Start of goalie Behaviour. Decides if the goalie has to go back to its own goal.

History:
* 05.12.14: Created (Marc Bestmann & Nils Rokita)
"""
from bitbots_body_behaviour.body.decisions.goalie.ball_seen import BallSeenGoalie
from bitbots_body_behaviour.body.decisions.common.go_to_duty_position import GoToDutyPosition
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement


class GoaliePositionDecision(AbstractDecisionElement):
    def perform(self, connector, reevaluate=False):
        if connector.blackboard.get_goalie_out_of_goal():
            return self.push(GoToDutyPosition)

        return self.push(BallSeenGoalie)
