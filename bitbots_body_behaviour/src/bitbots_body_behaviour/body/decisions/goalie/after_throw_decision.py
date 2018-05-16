# -*- coding:utf-8 -*-
"""
InGoal
^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from bitbots_body_behaviour.body.actions.throw import MIDDLE
from bitbots_body_behaviour.body.decisions.common.go_to_duty_position import GoToDutyPosition


class AfterThrowDecision(AbstractDecisionModule):
    """
    Decides how the robot will turn after it has thrown itself.
    """

    def __init__(self, connector, _):
        super(AfterThrowDecision, self).__init__(connector)
        self.relocateTurn = connector.config["Behaviour"]["Toggles"]["Goalie"]["relocateTurn"]
        self.anim_goalie_walkready = connector.config["animations"]["motion"]["goalie-walkready"]

    def perform(self, connector, reevaluate=False):
        direction = connector.blackboard.get_throw_direction()
        if direction == MIDDLE:
            connector.blackboard.delete_was_thrown()

        if self.relocateTurn:
            # Turn back to the right angle after throw
            return self.push(GoToDutyPosition)
        else:
            connector.blackboard.delete_was_thrown()
            return self.pop()
