# -*- coding:utf-8 -*-
"""
BallDangerous
^^^^^^^^^^^^^

.. moduleauthor:: Judith Hartfill (2hartfil@informatik.uni-hamburg.de)
                  Alexander Happel (2happel@informatik.uni-hamburg.de)
The robot decides if he ist exactly between Ball and Goal to protect it.
History:
* 19.08.14 Created (Judith Hartfill)
"""
import time

from bitbots_common.connector.connector import BodyConnector
from bitbots_common.util.math_utils import convert_uv2angular
from bitbots_body_behaviour.body.actions.plain_walk_action import PlainWalkAction
from bitbots_body_behaviour.body.actions.search import Search
from bitbots_body_behaviour.body.actions.wait import Wait
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class DefenderPositionDecider(AbstractDecisionModule):
    def __init__(self, connector: BodyConnector, _):
        super(DefenderPositionDecider, self).__init__(connector, _)
        self.timestamp_goal = 0
        self.wait_goal_time = 1

    def perform(self, connector, reevaluate=False):
        self.owngoal = connector.filtered_vision_capsule().get_local_goal_model_own_goal()
        # Damit es erstmal einen Wert gibt, um am anfang nicht durch 0 zu teilen etc
        # Startup defaults,
        if self.owngoal == (0.0, 0.0):
            self.owngoal = (-1000.0, 100.0)

        self.ball = connector.filtered_vision_capsule().get_local_goal_model_ball()
        if self.ball == (0.0, 0.0):
            self.ball = (2000.0, 1000.0)

        self.angularball = (convert_uv2angular(self.ball[0], self.ball[1]))

        # place inbetween goalie and nearest goalpost
        self.defenderPoint = connector.filtered_vision_capsule().get_local_goal_model_defender_point()

        # angle to own goal
        self.angularowngoal = (convert_uv2angular(self.defenderPoint[0], self.defenderPoint[1]))

        self.angularballgoal = abs(self.angularowngoal - self.angularball)
        if self.angularballgoal >= 180:
            self.angularballgoal = 360 - self.angularballgoal

        if self.timestamp_goal < (int(time.time()) - self.wait_goal_time):
            self.timestamp_goal = int(time.time())
        # Search first for the ball
        if self.ball == (2000.0, 1000.0):
            self.debug("Search for ball")
            return self.push(Search)

        # Search first for goal
        elif self.owngoal == (-1000.0, 100.0):
            self.debug("search for goal")
            return self.push(Search)

        connector.blackboard_capsule().set_priorities(ball_priority=5,
                                                      line_priority=0,
                                                      own_goal_priority=0,
                                                      enemy_goal_priority=10,
                                                      align_priority=0)

        if -90 < self.angularball < 90:
            if (15 < self.angularballgoal < 165) or (- 195 > self.angularballgoal):
                # #Todo directions in plain walk action seam to be twistet
                # light angle to counteract bad sideward movement
                return self.push(PlainWalkAction, [
                    [WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_RIGHT, WalkingCapsule.SLOW_SIDEWARDS_LEFT, 10]])
            elif (15 > self.angularballgoal > -165) or (195 < self.angularballgoal):
                return self.push(PlainWalkAction, [
                    [WalkingCapsule.ZERO, WalkingCapsule.SLOW_ANGULAR_LEFT, WalkingCapsule.SLOW_SIDEWARDS_RIGHT, 10]])
            else:
                return self.push(Wait)
        else:
            return self.push(Wait)

    def get_reevaluate(self):
        return True
