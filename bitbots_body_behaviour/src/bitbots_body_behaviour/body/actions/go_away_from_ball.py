# -*- coding:utf-8 -*-
"""
GotToBallPathfinding
^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from math import atan2
from bitbots_body_behaviour.body.actions.go_to import GoToRelativePosition
from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class GoAwayFromBall(AbstractActionModule):
    """Goes away from the ball"""
    def perform(self, connector, reevaluate=False):
        ball_relative = connector.personal_model.get_ball_relative()
        point = (-ball_relative[0],
                 -ball_relative[1],
                 atan2(ball_relative[1], ball_relative[0]))
        return self.push(GoToRelativePosition, point)
