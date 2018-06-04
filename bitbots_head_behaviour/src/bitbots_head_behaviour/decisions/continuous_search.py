# -*- coding:utf-8 -*-
"""
ContinuousSearch
^^^^^^^^^^^^^^^^

.. moduleauthor:: Nils <0rokita@informatik.uni-hamburg.de>

This Modul performs a continuous search with the head. In the standard case it works with a simple fixed pattern,
so only in special cases the ball will be tracked.

"""
import rospy

from bitbots_head_behaviour.actions.look_at import LookAtRelativePoint
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from humanoid_league_msgs.msg import TeamData


class ContinuousSearch(AbstractDecisionModule):
    def __init__(self, connector, outcomes=()):
        super(ContinuousSearch, self).__init__(connector)
        self.pattern_pos = 0
        self.goalie_pattern = connector.config["Head"]["SearchPattern"]["goalie"]
        self.defender_pattern = connector.config["Head"]["SearchPattern"]["defender"]
        self.center_pattern = connector.config["Head"]["SearchPattern"]["center"]
        self.ball_pattern = connector.config["Head"]["SearchPattern"]["ball"]
        self.default_pattern = connector.config["Head"]["SearchPattern"]["ball"]
        self.current_pattern = self.default_pattern
        self.last_pattern = self.current_pattern
        rospy.logdebug("Start new Search")

    def perform(self, connector, reevaluate=False):
        self.repr_data = {}
        self.set_pattern(connector)

        pos = self.pattern_pos
        self.repr_data["pattern_pos"] = pos
        rospy.logdebug("Pattern pos" + str(pos))
        # Increment the to be reached postion with wrap around
        self.pattern_pos = (pos + 1) % len(self.current_pattern)
        point = (self.current_pattern[pos][0], self.current_pattern[pos][1], 0)
        return self.push(LookAtRelativePoint, point)

    def set_pattern(self, connector):
        self.last_pattern = self.current_pattern
        role = connector.team_data.get_role()

        if connector.head.is_ball_tracking_still_active:
            # we only come here if the continuousSearch is called by the SearchForBall Module
            # in this case we want to search the ball
            self.repr_data["pattern"] = "ball_pattern"
            self.current_pattern = self.ball_pattern
        elif role == TeamData.ROLE_GOALIE:
            self.repr_data["pattern"] = "goalie_pattern"
            self.current_pattern = self.goalie_pattern
        elif role == TeamData.ROLE_DEFENDER:
            self.repr_data["pattern"] = "defender_pattern"
            self.current_pattern = self.defender_pattern
        elif role == TeamData.ROLE_OTHER:
            self.repr_data["pattern"] = "center_pattern"
            self.current_pattern = self.center_pattern
        else:
            self.repr_data["pattern"] = "default_pattern"
            self.current_pattern = self.default_pattern

        # reset the pattern pos if we changed our pattern
        if not self.last_pattern == self.current_pattern:
            self.pattern_pos = 0
