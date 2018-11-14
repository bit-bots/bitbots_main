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
    def __init__(self, connector, args=10):
        super(Wait, self).__init__(connector)
        self.time = rospy.get_time() + args

    def perform(self, connector, reevaluate=False):
        if connector.world_model.ball_seen():
            connector.blackboard.set_head_duty(HeadMode.BALL_MODE)

        #todo this was an old push which is not correct, since it is an action
        #self.push(Stand)
        if self.time < rospy.get_time():
            self.pop()
