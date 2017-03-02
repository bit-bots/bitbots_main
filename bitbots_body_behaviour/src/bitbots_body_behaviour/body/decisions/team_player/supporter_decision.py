"""
SupporterDecision
^^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import time

from body.actions.go_to_ball_pathfinding import GoToBallPathfinding
from body.actions.search import Search
from body.actions.wait import Wait
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from bitbots_common.connector.connector import BodyConnector


class SupporterDecision(AbstractDecisionModule):
    def __init__(self, connector, args=None):
        super().__init__(connector, args)

    def perform(self, connector: BodyConnector, reevaluate=False):

        if not (connector.vision.ball_seen() or
                time.time() - connector.vision.ball_last_seen() < 1):
            return self.push(Search)

        if connector.vision.get_ball_distance() < 0.9:
            return self.push(Wait, 9999999)
        elif connector.vision.get_ball_distance() > 1.1:
            return self.push(GoToBallPathfinding)
        else:
            return self.push(Wait, 9999999)

    def get_reevaluate(self):
        return True
