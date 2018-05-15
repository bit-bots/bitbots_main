# -*- coding:utf-8 -*-
"""
CloseBall
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>
"""
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule

from math import atan2
from bitbots_body_behaviour.body.actions.align_on_ball import AlignOnBall
from bitbots_body_behaviour.body.actions.go_to import GoToBall
from bitbots_body_behaviour.body.decisions.common.kick_decision import KickDecisionPenaltyKick
from bitbots_body_behaviour.body.decisions.common.stands_correct_decision import StandsCorrectDecision
from bitbots_body_behaviour.body.decisions.penalty.penalty_first_kick import PenaltyFirstKick
from humanoid_league_msgs.msg import HeadMode


class AbstractCloseBall(AbstractDecisionModule):
    """
    Test if the ball is in kick distance
    """

    def __init__(self, connector, _):
        super(AbstractCloseBall, self).__init__(connector)
        self.last_goalie_dist = 0
        self.last_goalie_dist_time = 0
        self.max_kick_distance = connector.config["Body"]["Fieldie"]["kickDistance"]
        self.min_kick_distance = connector.config["Body"]["Fieldie"]["minKickDistance"]
        self.config_kickalign_v = connector.config["Body"]["Fieldie"]["kickAlign"]

    def perform(self, connector, reevaluate=False):
        # When the ball is seen, the robot should switch between looking to the ball and the goal
        connector.blackboard.set_head_duty(HeadMode.BALL_GOAL_TRACKING)
        # if the robot is near to the ball
        if self.min_kick_distance < connector.personal_model.get_ball_relative()[0] <= self.max_kick_distance \
                and connector.personal_model.get_ball_distance() <= self.max_kick_distance * 5.0:
            # TODO config
            self.action(connector)
        else:
            self.go(connector)

    def action(self, connector):
        return self.push(StandsCorrectDecision)

    def go(self, connector):
        goal_relative = connector.personal_model.get_goal_relative()
        direction = atan2(goal_relative[1], goal_relative[0])
        connector.pathfinding.go_to(connector.personal_model.get_ball_relative()[0],
                                    connector.personal_model.get_ball_relative()[1],
                                    direction)

    def get_reevaluate(self):
        return True


class CloseBallCommon(AbstractCloseBall):
    pass


class CloseBallPenaltyKick(AbstractCloseBall):
    def __init__(self, connector):
        super(CloseBallPenaltyKick, self).__init__(connector)
        self.toggle_direct_penalty = connector.config["Behaviour"]["Toggles"]["PenaltyFieldie"]["directPenaltyKick"]
        self.use_special_pathfinding = connector.config["Behaviour"]["Toggles"]["PenaltyFieldie"]["useSpecialPathfinding"]

    def action(self, connector):
        if not self.toggle_direct_penalty and not connector.blackboard.get_first_kick_done():
            # todo first kick müsste eigenlich nicht extra abgecheckt werden, später raus nehmen
            # the penalty kick is different for now
            return self.push(PenaltyFirstKick)
        else:
            if abs(connector.personal_model.get_ball_relative()[1]) > self.config_kickalign_v:
                return self.push(AlignOnBall)
            else:
                return self.push(KickDecisionPenaltyKick)

    def go(self, connector):
        return self.push(GoToBall)


class CloseBallGoalie(AbstractCloseBall):
    def perform(self, connector, reevaluate=False):
        if connector.blackboard_capsule().has_goalie_kicked():
            return self.interrupt()
        super(AbstractCloseBall, self).perform(connector, reevaluate)  # perform an normal CloseBall
