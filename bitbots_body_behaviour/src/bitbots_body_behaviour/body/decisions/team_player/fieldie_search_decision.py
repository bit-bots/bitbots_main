# -*- coding:utf-8 -*-
"""
FieldieSearchDecision
^^^^^^^^^^^^^^^^^^^^

This Module decides if the Fieldie should Search or Walk/Turn to Find the Ball

The Search is continued until someone interrupts it by reevaluate or interrupt

History:
''''''''

* 06.12.14 Created (Nils Rokita)

"""
import rospy

from bitbots_body_behaviour.body.actions.search import Search
from bitbots_body_behaviour.body.actions.go_to import GoToCenterpoint, GoToRelativePosition

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class FieldieSearchDecision(AbstractDecisionModule):
    def __init__(self, connector, _):
        super(FieldieSearchDecision, self).__init__(connector, _)
        self.start_time = rospy.get_time()
        self.turn_wait_time = connector.config["Body"]["Fieldie"]["searchWaitTime"]
        self.turns_before_going_to_center_point = connector.config["Body"]["Fieldie"]["turnCenterpointTime"]
        self.turn_angle = connector.config["Body"]["Fieldie"]["searchingTurnAngularAbsolute"]
        self.counter = 0
        self.at_centerpoint = False

    def perform(self, connector, reevaluate=False):
        # if we have looked too long in the same direction without seeing we want to walk
        if rospy.get_time() - connector.pathfinding.arrive_time > self.turn_wait_time:
            if self.counter >= self.turns_before_going_to_center_point and not self.at_centerpoint:
                self.push(GoToCenterpoint)
                self.at_centerpoint = True
            else:
                # reset the timer and increment counter
                if not connector.pathfinding.is_walking_active():
                    self.counter += 1
                self.push(GoToRelativePosition, (0, 0, self.turn_angle))
        else:
            # Just Search
            self.push(Search)
