# -*- coding:utf-8 -*-
"""
AlignOnBall
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
from bitbots_body_behaviour.actions.go_to import GoToRelativePosition
from humanoid_league_msgs.msg import HeadMode


class AlignOnBall(AbstractActionElement):
    def perform(self, connector, reevaluate=False):
        # When positioning, the robot should only look to the ball
        connector.blackboard.set_head_duty(HeadMode.BALL_MODE)

        # Position yourself about 20 centimeters behind the ball
        # TODO: parameter
        ball_u, ball_v = connector.world_model.get_ball_position_uv()
        point = (ball_u - 0.2, ball_v[1], 0)
        return self.push(GoToRelativePosition, point)
