"""
SupporterDecision
^^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import time

from body.actions.search import Search
from body.actions.wait import Wait
from stackmachine.abstract_decision_module import AbstractDecisionModule


class SupporterDecision(AbstractDecisionModule):
    def __init__(self, _):
        super(SupporterDecision, self).__init__()

    def perform(self, connector, reevaluate=False):

        if not (connector.raw_vision_capsule().ball_seen() or
                time.time() - connector.raw_vision_capsule().get_last_seen("Ball") < 1):
            return self.push(Search)

        if connector.raw_vision_capsule().get_ball_info("distance") < 900:
            return self.push(Wait, 9999999)
        elif connector.raw_vision_capsule().get_ball_info("distance") > 1100:
            return self.push(GoToBall)
        else:
            return self.push(Wait, 9999999)

    def get_reevaluate(self):
        return True
