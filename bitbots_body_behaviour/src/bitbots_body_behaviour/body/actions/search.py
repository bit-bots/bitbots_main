# -*- coding:utf-8 -*-
"""
Search
^^^^^^
"""
from bitbots_stackmachine.abstract_action_module import AbstractActionModule
from humanoid_league_msgs.msg import HeadMode


class Search(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        # Tell the head that it should search for the ball
        connector.blackboard.set_head_duty(HeadMode.BALL_MODE)
        self.pop()


class StopAndSearch(Search):
    def perform(self, connector, reevaluate=False):
        connector.walking.stop_walking()
        super(StopAndSearch, self).perform(connector, reevaluate)
