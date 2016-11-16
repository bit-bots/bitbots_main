# -*- coding:utf-8 -*-
"""
ContiniousSearch
^^^^^^^^^^^^^^^^

.. moduleauthor:: Nils <0rokita@informatik.uni-hamburg.de>

This Modul performs a continious search with the head. In the standard case it works with a simple fixed pattern,
so only in special cases the ball will be tracked.

History:

* 19.08.14: Created (Nils)

"""

from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.head.actions.head_to_pan_tilt import HeadToPanTilt
from bitbots.util.config import get_config


class ContiniousSearch(smach.State):

    def __init__(self, outcomes=['outcome1', 'outcome2']):
        self.pattern_pos = 0
        config = get_config()
        self.goalie_pattern = config["Behaviour"]["Common"]["SearchPattern"]["goalie"]
        self.defender_pattern = config["Behaviour"]["Common"]["SearchPattern"]["defender"]
        self.center_pattern = config["Behaviour"]["Common"]["SearchPattern"]["center"]
        self.ball_pattern = config["Behaviour"]["Common"]["SearchPattern"]["ball"]
        self.default_pattern = config["Behaviour"]["Common"]["SearchPattern"]["ball"]  # todo default <-> Ball pattern
        self.current_pattern = self.default_pattern
        self.last_pattern = self.current_pattern

    def perform(self, connector, reevaluate=False):
        self.set_pattern(connector)

        pos = self.pattern_pos
        # Increment the to be reached postion with wrap around
        self.pattern_pos = (pos + 1) % len(self.current_pattern)
        return self.push(HeadToPanTilt, self.current_pattern[pos])

    def set_pattern(self, connector):
        self.last_pattern = self.current_pattern
        duty = connector.get_duty()
        role = connector.team_data_capsule().get_role()

        if connector.blackboard_capsule().is_ball_tracking_still_active():
            # we only come here if the continousSearch is called by the SearchForBall Module
            # in this case we want to search the ball
            self.current_pattern = self.ball_pattern
        elif duty == "Goalie":
            self.current_pattern = self.goalie_pattern
        elif role == "Defender":
            self.current_pattern = self.defender_pattern
        elif role == "Center":
            self.current_pattern = self.center_pattern
        else:
            self.current_pattern = self.default_pattern

        # reset the pattern pos if we changed our pattern
        if not self.last_pattern == self.current_pattern:
            self.pattern_pos = 0
