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
        ball_u, ball_v = connector.world_model.get_ball_position_uv()
        point = (-ball_u,
                 -ball_v,
                 atan2(ball_v, ball_u))
        return self.push(GoToRelativePosition, point)
