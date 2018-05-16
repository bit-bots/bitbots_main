# -*- coding:utf-8 -*-
"""
SupporterDecision
^^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import rospy

from bitbots_body_behaviour.body.actions.go_to import GoToBall
from bitbots_body_behaviour.body.actions.search import Search
from bitbots_body_behaviour.body.actions.wait import Wait
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class SupporterDecision(AbstractDecisionModule):
    def __init__(self, connector, args=None):
        super(SupporterDecision, self).__init__(connector, args)

    def perform(self, connector, reevaluate=False):

        if not (connector.personal_model.ball_seen() or
                rospy.get_time() - connector.personal_model.ball_last_seen() < 1):
            return self.push(Search)

        if connector.personal_model.get_ball_distance() > 1.1:
            return self.push(GoToBall)
        else:
            return self.push(Wait)

    def get_reevaluate(self):
        return True
