# -*- coding:utf-8 -*-
"""
FocusBall
^^^^^^^^^^

.. moduleauthor:: Fabian Fiedler <0fiedler@informatik.uni-hamburg.de>

History:

* 12.03.14 erstellt

Just focusing the ball.

"""
from bitbots.modules.abstract.abstract_action_module import AbstractActionModule


class FocusBall(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().schedule_ball_tracking()
        return self.pop()
