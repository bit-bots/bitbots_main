# -*- coding:utf-8 -*-
"""
Wait
^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 20.3.14 created (Martin Poppinga)

Just waits fr something (i.e. that preconditions will be fullfilled)
"""
import time

from bitbots.modules.abstract.abstract_action_module import AbstractActionModule


class Wait(AbstractActionModule):
    def __init__(self, args=99999999):
        super(Wait, self).__init__()
        if args is None:
            args = 10
        self.time = time.time() + args

    def perform(self, connector, reevaluate=False):
        if connector.raw_vision_capsule().ball_seen():
            connector.blackboard_capsule().schedule_ball_tracking()
        connector.walking_capsule().stop_walking()
        if self.time > time.time():
            self.pop()



