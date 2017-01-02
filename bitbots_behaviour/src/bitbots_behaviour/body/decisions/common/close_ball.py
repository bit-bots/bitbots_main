# -*- coding:utf-8 -*-
"""
CloseBall
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 29.11.13: Created (Martin Poppinga)
"""

from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.align_on_ball import AlignOnBall
from bitbots.modules.behaviour.body.actions.go_to_ball_intelligent import GoToBallIntelligent
from bitbots.modules.behaviour.body.decisions.common.stands_correct_decision import StandsCorrectDecision
from bitbots.modules.behaviour.body.decisions.penalty.penalty_first_kick import PenaltyFirstKick
from bitbots.util import get_config
from bitbots.modules.behaviour.body.decisions.common.kick_decision import KickDecisionPenaltyKick
from bitbots.modules.behaviour.body.decisions.common.kick_decision import KickDecisionThrowIn
from bitbots.modules.behaviour.body.actions.go_to_ball import GoToBallPenaltykick

config = get_config()


class AbstractCloseBall(AbstractDecisionModule):
    """
    Test if the ball is in kick distance
    """

    def __init__(self, _):
        super(AbstractCloseBall, self).__init__()
        self.last_goalie_dist = 0
        self.last_goalie_dist_time = 0
        self.max_kick_distance = config["Behaviour"]["Fieldie"]["kickDistance"]
        self.min_kick_distance = config["Behaviour"]["Fieldie"]["minKickDistance"]
        self.config_kickalign_v = config["Behaviour"]["Fieldie"]["kickAlign"]

    def perform(self, connector, reevaluate=False):
        # if the robot is near to the ball
        if self.min_kick_distance < connector.raw_vision_capsule().get_ball_info("u") <= self.max_kick_distance \
                and connector.raw_vision_capsule().get_ball_info("distance") <= self.max_kick_distance * 5.0:
            # TODO config
            self.action(connector)
        else:
            self.go()

    def action(self, connector):
        return self.push(StandsCorrectDecision)

    def go(self):
        return self.push(GoToBallIntelligent)

    def get_reevaluate(self):

        return True


class CloseBallCommon(AbstractCloseBall):
    pass


class CloseBallThrowIn(AbstractCloseBall):
    def action(self, connector):
        return self.push(KickDecisionThrowIn)


class CloseBallPenaltyKick(AbstractCloseBall):  # todo not yet refactored 6.12.14.
    def __init__(self, _):
        super(CloseBallPenaltyKick, self).__init__(_)
        self.toggle_direct_penalty = config["Behaviour"]["Toggles"]["PenaltyFieldie"]["directPenaltyKick"]
        self.use_special_pathfinfing = config["Behaviour"]["Toggles"]["PenaltyFieldie"]["useSpecialPathfinding"]

    def action(self, connector):
        if not self.toggle_direct_penalty and not connector.blackboard_capsule().get_first_kick_done():
                # todo first kick müsste eigenlich nicht extra abgecheckt werden, später raus nehmen
            # the penatly kick is different for now
            return self.push(PenaltyFirstKick)

        else:
            if abs(connector.raw_vision_capsule().get_ball_info(
                    "v")) > self.config_kickalign_v:  # todo wieder gefilterte daten verwenden

                return self.push(AlignOnBall)
            else:
                return self.push(KickDecisionPenaltyKick)

    def go(self):
        if self.use_special_pathfinfing:
            return self.push(GoToBallPenaltykick)
        else:
            return self.push(GoToBallIntelligent)


class CloseBallGoalie(AbstractCloseBall):
    def perform(self, connector, reevaluate=False):
        if connector.blackboard_capsule().has_goalie_kicked():
            return self.interrupt()
        super(AbstractCloseBall, self).perform(connector, reevaluate)  # perform an normal closeball
