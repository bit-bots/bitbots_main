# -*- coding:utf-8 -*-
"""
Search
^^^^^^
"""
from bitbots_body_behaviour.actions.go_to import Stand
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode


class Search(AbstractActionElement):
    def perform(self, connector, reevaluate=False):
        # Tell the head that it should search for the ball
        connector.blackboard.set_head_duty(HeadMode.BALL_MODE)
        self.pop()


class StopAndSearch(Search):
    def perform(self, connector, reevaluate=False):
        connector.blackboard.set_head_duty(HeadMode.BALL_MODE)
        self.push(Stand)
        self.pop()
