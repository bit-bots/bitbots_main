# -*- coding:utf-8 -*-
"""
InGoal
^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 29.11.13: Created (Martin Poppinga)
"""
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.animation_play_action import AnimationPlayAction
from bitbots.modules.behaviour.body.actions.throw import MIDDLE
from bitbots.modules.behaviour.body.decisions.goalie.turn_after_throw import TurnAfterThrow
from bitbots.util import get_config

config = get_config()


class AfterThrowDecision(AbstractDecisionModule):
    """
    Decides how the robot will turn after it has thrown itself.
    """

    def __init__(self, _):
        super(AfterThrowDecision, self).__init__()
        self.relocateTurn = config["Behaviour"]["Toggles"]["Goalie"]["relocateTurn"]
        self.anim_goalie_walkready = config["animations"]["motion"]["goalie-walkready"]

    def perform(self, connector, reevaluate=False):

        richtung = connector.blackboard_capsule().get_throw_direction()
        if richtung == MIDDLE:
            connector.blackboard_capsule().delete_was_thrown()
            self.push(AnimationPlayAction, self.anim_goalie_walkready)

        if self.relocateTurn:
            return self.push(TurnAfterThrow)
        else:
            connector.blackboard_capsule().delete_was_thrown()
            return self.pop()
