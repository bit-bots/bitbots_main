# -*- coding:utf-8 -*-
"""
AlignOnBall
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from abstract.abstract_action_module import AbstractActionModule


class AlignOnBall(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().schedule_ball_tracking()

        connector.walking.start_walking_plain(
            -1,
            0,
            self.sign(connector.raw_vision_capsule().get_ball_info("v") * 2)
        )
    #todo definieren wann wer dem Walking dinge mitteilt