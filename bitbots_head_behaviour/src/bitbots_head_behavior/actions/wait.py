# -*- coding:utf-8 -*-
"""
Wait
^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Just waits for something (i.e. that preconditions will be fullfilled)
"""
import rospy

from bitbots_dsd.abstract_action_element import AbstractActionElement


class Wait(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(Wait, self).__init__(blackboard, dsd)
        self.time = rospy.get_time() + float(parameters['time'])

    def perform(self, reevaluate=False):
        if self.time < rospy.get_time():
            self.pop()
