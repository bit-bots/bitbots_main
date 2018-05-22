# -*- coding:utf-8 -*-
"""
Wait
^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Just waits for something (i.e. that preconditions will be fullfilled)
"""
import rospy

from bitbots_body_behaviour.body.actions.go_to import GoToRelativePosition
from bitbots_stackmachine.abstract_action_module import AbstractActionModule
from humanoid_league_msgs.msg import HeadMode


class Wait(AbstractActionModule):
    def __init__(self, connector, args=10):
        super(Wait, self).__init__(connector)
        self.time = rospy.get_time() + args

    def perform(self, connector, reevaluate=False):
        if connector.personal_model.ball_seen():
            connector.blackboard.set_head_duty(HeadMode.BALL_MODE)

        self.push(GoToRelativePosition, (0, 0, 0))
        if self.time > rospy.get_time():
            self.pop()
