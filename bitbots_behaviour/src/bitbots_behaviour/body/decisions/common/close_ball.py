"""
CloseBall
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>
"""
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule

import rospy
from body.actions.align_on_ball import AlignOnBall
from body.actions.go_to_ball_pathfinding import GoToBallPathfinding
from body.decisions.common.kick_decision import KickDecisionPenaltyKick
from body.decisions.common.stands_correct_decision import StandsCorrectDecision
from body.decisions.penalty.penalty_first_kick import PenaltyFirstKick
from model.connector import Connector


class AbstractCloseBall(AbstractDecisionModule):
    """
    Test if the ball is in kick distance
    """

    def __init__(self, connector: Connector, _):
        super(AbstractCloseBall, self).__init__(connector)
        self.last_goalie_dist = 0
        self.last_goalie_dist_time = 0
        self.max_kick_distance = connector.config["Fieldie"]["kickDistance"]
        self.min_kick_distance = connector.config["Fieldie"]["minKickDistance"]
        self.config_kickalign_v = connector.config["Fieldie"]["kickAlign"]

    def perform(self, connector: Connector, reevaluate=False):
        # if the robot is near to the ball
        if self.min_kick_distance < connector.vision.get_ball_relative[0] <= self.max_kick_distance \
                and connector.vision.get_ball_distance <= self.max_kick_distance * 5.0:
            # TODO config
            self.action(connector)
        else:
            self.go()

    def action(self, connector):
        return self.push(StandsCorrectDecision)

    def go(self):
        return self.push(GoToBallPathfinding)

    def get_reevaluate(self):
        return True


class CloseBallCommon(AbstractCloseBall):
    pass


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
            return self.push(GoToBallPathfinding)


class CloseBallGoalie(AbstractCloseBall):
    def perform(self, connector, reevaluate=False):
        if connector.blackboard_capsule().has_goalie_kicked():
            return self.interrupt()
        super(AbstractCloseBall, self).perform(connector, reevaluate)  # perform an normal closeball
