# -*- coding:utf-8 -*-
"""
BallSeen
^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 29.11.13: Created (Martin Poppinga)
* 12.03.14: Changed to use Config (Marc)
"""
import time
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.decisions.goalie.ball_dangerous import BallDangerous
from bitbots.modules.behaviour.body.decisions.common.close_ball import CloseBallPenaltyKick, CloseBallCommon
from bitbots.modules.behaviour.body.decisions.team_player.fieldie_search_decision import FieldieSearchDecision
from bitbots.modules.behaviour.body.actions.search import Search
from bitbots.modules.behaviour.body.decisions.common.close_ball import CloseBallThrowIn
from bitbots.util import get_config

config = get_config()


class AbstractBallSeen(AbstractDecisionModule):
    """
    Entscheidet ob der Ball gesehen wurde bzw. ob die Informationen zuverlässig genug sind
    Decides if the ball was seen rspectively if the information is  authentic enough.
    """

    def __init__(self, _):
        super(AbstractBallSeen, self).__init__()
        self.max_ball_time = config["Behaviour"]["Common"]["maxBallTime"]
        self.anim_goali_walkready = config["animations"]["motion"]["goalie-walkready"]

    def perform(self, connector, reevaluate=False):

        if connector.raw_vision_capsule().ball_seen() or \
                ((time.time() - connector.raw_vision_capsule().get_last_seen("Ball")) < self.max_ball_time):
            return self.has_ball_seen(connector)
        else:
            return self.ball_not_seen(connector)

    def get_reevaluate(self):
        return True

    def has_ball_seen(self, connector):
        raise NotImplementedError

    def ball_not_seen(self, connector):
        raise NotImplementedError


class BallSeenGoalie(AbstractBallSeen):
    def perform(self, connector, reevaluate=False):

        if time.time() - connector.blackboard_capsule().get_confirmed_ball() < 2:
            return self.has_ball_seen(connector)
        else:
            return self.ball_not_seen(connector)

    def has_ball_seen(self, connector):
        return self.push(BallDangerous)

    def ball_not_seen(self, connector):
        return self.push(Search)


class BallSeenFieldie(AbstractBallSeen):
    """
    def perform(self, connector, reevaluate=False):

        if connector.raw_vision_capsule().ball_seen() or \
                ((time.time() - connector.blackboard_capsule().get_confirmed_ball() < 5 and
                connector.filtered_vision_capsule().get_local_goal_model_ball_distance() > 1000)
            or (time.time() - connector.blackboard_capsule().get_confirmed_ball() < 2 and
                connector.filtered_vision_capsule().get_local_goal_model_ball_distance() < 1000)):
            return self.has_ball_seen(connector)
        else:
            return self.ball_not_seen(connector)
    """

    def has_ball_seen(self, connector):
        return self.push(CloseBallCommon)

    def ball_not_seen(self, connector):
        return self.push(FieldieSearchDecision)


class BallSeenThrowIn(AbstractBallSeen):
    def has_ball_seen(self, connector):
        return self.push(CloseBallThrowIn)

    def ball_not_seen(self, connector):
        return self.push(Search)


class BallSeenPenaltyKick(AbstractBallSeen):  # todo not yet refactored 6.12.14.
    def has_ball_seen(self, connector):
        return self.push(CloseBallPenaltyKick)

    def ball_not_seen(self, connector):
        return self.push(Search)
