# -*- coding:utf-8 -*-
"""
AlignOnBall
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 4.3.14: Created (Martin Poppinga)

Der Roboter soll sich zum schießen am Ball ausrichten, und danach am Besten in richtung des gegnerischen
Tores stehen, sodass er ihn reinschieße kann.
The robot will repositionate on the Ball. So he can shoot the ball.

"""
from bitbots.modules.abstract.abstract_action_module import AbstractActionModule


class AlignOnBall(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().schedule_ball_tracking()

        connector.walking_capsule().start_walking_plain(
            -1,
            0,
            self.sign(connector.raw_vision_capsule().get_ball_info("v") * 2)
        )
