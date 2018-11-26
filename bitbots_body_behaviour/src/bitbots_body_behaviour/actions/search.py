# -*- coding:utf-8 -*-
"""
Search
^^^^^^
"""
from bitbots_body_behaviour.actions.go_to import Stand
from bitbots_dsd.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode


class Search(AbstractActionElement):
    def perform(self, reevaluate=False):
        # Tell the head that it should search for the ball
        blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE)
        self.pop()


class StopAndSearch(Search):
    def perform(self, reevaluate=False):
        blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE)
        self.push(Stand)
        self.pop()
