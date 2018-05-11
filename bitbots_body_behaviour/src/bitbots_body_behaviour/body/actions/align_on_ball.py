# -*- coding:utf-8 -*-
"""
AlignOnBall
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from bitbots_stackmachine.abstract_action_module import AbstractActionModule
from bitbots_body_behaviour.body.actions.go_to import GoToRelativePosition
from humanoid_league_msgs.msg import HeadMode


class AlignOnBall(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        # When positioning, the robot should only look to the ball
        connector.blackboard.set_head_duty(HeadMode.BALL_MODE)

        # Position yourself about 20 centimeters behind the ball
        # TODO: parameter
        point = (connector.personal_model.get_ball_relative()[0] - 0.2,
                 connector.personal_model.get_ball_relative()[1],
                 0)
        return self.push(GoToRelativePosition, point)
