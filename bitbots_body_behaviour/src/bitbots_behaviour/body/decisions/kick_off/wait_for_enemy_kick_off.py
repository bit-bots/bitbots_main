# -*- coding:utf-8 -*-
"""
WaitForEnemyKickOff
^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:
* 06.01.15: Created (Marc Bestmann)
"""
import time

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class WaitForEnemyKickOff(AbstractDecisionModule):
    def __init__(self):
        super(WaitForEnemyKickOff, self).__init__()
        self.start_time = time.time()

    def perform(self, connector, reevaluate=False):
        if time.time() - self.start_time > 9:
            return self.interrupt()
        else:
            ball_x, ball_y = connector.world_model_capsule().get_ball_position_xy()

            if ball_x > 100 or ball_y > 100:  # ball moved from middle point
                # todo the world model is maybe not exact enought
                # todo save the information, that we can play
                return self.interrupt()
            else:
                return self.push(Wait, 0.1)
