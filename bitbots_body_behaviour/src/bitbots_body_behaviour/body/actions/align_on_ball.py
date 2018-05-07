# -*- coding:utf-8 -*-
"""
AlignOnBall
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class AlignOnBall(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        connector.blackboard.set_head_duty("BALL_MODE")

        connector.walking.start_walking_plain(
            -0.04,
            0,
            self.sign(connector.personal_model.get_ball_relative()[1] * 0.02)
        )
