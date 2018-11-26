# -*- coding:utf-8 -*-
"""
Wait
^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Just waits for something (i.e. that preconditions will be fullfilled)
"""
import rospy

from bitbots_body_behaviour.actions.go_to import Stand
from bitbots_dsd.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode


class Wait(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(Wait, self).__init__(blackboard, dsd)
        self.time = rospy.get_time() + parameters

    def perform(self, reevaluate=False):
        if self.blackboard.world_model.ball_seen():
            self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE)

        #todo this was an old push which is not correct, since it is an action
        #self.push(Stand)
        if self.time < rospy.get_time():
            self.pop()
