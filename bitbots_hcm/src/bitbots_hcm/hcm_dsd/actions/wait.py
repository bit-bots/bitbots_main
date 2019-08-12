# -*- coding:utf-8 -*-
"""
Wait
^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Just waits for something (i.e. that preconditions will be fullfilled)
"""
import rospy

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class Wait(AbstractActionElement):
    """
    This action waits a specified time before it pops itself
    """

    def __init__(self, blackboard, dsd, parameters=None):
        """
        :param parameters['time']: Time to wait in seconds
        """
        super(Wait, self).__init__(blackboard, dsd)
        self.time = rospy.get_time() + float(parameters['time'])

    def perform(self, reevaluate=False):
        """
        Only pop when the wait-time has elapsed
        """

        if self.time < rospy.get_time():
            self.pop()
