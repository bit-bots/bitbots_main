"""
Wait
^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Just waits for something (i.e. that preconditions will be fullfilled)
"""
from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement


class Wait(AbstractHCMActionElement):
    """
    This action does nothing. If a time is given, it will wait for that time before it pops itself.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        """
        :param parameters['time']: Time to wait in seconds
        """
        super().__init__(blackboard, dsd, parameters)
        self.duration = parameters.get("time", None)
        self.start_time = self.blackboard.node.get_clock().now().nanoseconds / 1e9

    def perform(self, reevaluate=False):
        """
        Only pop when the wait-time has elapsed
        """

        # Return directly if we want to wait infinitely
        if self.duration is None:
            return

        # Pop if the time has elapsed
        if self.blackboard.node.get_clock().now().nanoseconds / 1e9 > self.start_time + self.duration:
            self.pop()
