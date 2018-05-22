# -*- coding:utf-8 -*-
"""
BallDangerous
^^^^^^^^^^^^^

.. moduleauthor:: Judith Hartfill (2hartfil@informatik.uni-hamburg.de)
                  Alexander Happel (2happel@informatik.uni-hamburg.de)
The robot decides if he ist exactly between Ball and Goal to protect it.
History:
* 19.08.14 Created (Judith Hartfill)
"""
import rospy
import math

from bitbots_body_behaviour.body.actions.search import Search
from bitbots_body_behaviour.body.actions.go_to import GoToAbsolutePosition
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class DefenderPositionDecider(AbstractDecisionModule):
    def __init__(self, connector, _):
        super(DefenderPositionDecider, self).__init__(connector, _)
        self.timestamp_goal = 0
        self.wait_goal_time = 1

    def perform(self, connector, reevaluate=False):
        if not connector.world_model.ball_seen():
            return self.push(Search)

        ball = connector.world_model.get_ball_position_xy()
        goal = connector.world_model.get_own_goal_center_xy()

        # Go between the ball and the goal and look at the ball
        defender_position = ((ball[0] + goal[0]) / 2,
                             (ball[1] + goal[1]) / 2,
                             math.atan2(goal[1] - ball[1], goal[0] - ball[0]))
        return self.push(GoToAbsolutePosition, defender_position)

    def get_reevaluate(self):
        return True
