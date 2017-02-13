"""
Wait
^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Just waits for something (i.e. that preconditions will be fullfilled)
"""
import time

from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class Wait(AbstractActionModule):
    def __init__(self, args=99999999):
        super(Wait, self).__init__()
        if args is None:
            args = 10
        self.time = time.time() + args

    def perform(self, connector, reevaluate=False):
        if connector.vision.ball_seen():
            connector.blackboard.schedule_ball_tracking()

        connector.walking.stop_walking()
        if self.time > time.time():
            self.pop()
