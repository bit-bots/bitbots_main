# -*- coding:utf-8 -*-
"""
SearchForBall
^^^^^^^^^^^^^

Lets the head search only for the ball

History:


"""
import rospy
import math

from bitbots_head_behaviour.actions.head_to_pan_tilt import HeadToPanTilt
from bitbots_head_behaviour.decisions.continuous_search import ContinuousSearch
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class AbstractSearchForObject(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        pass

    def __init__(self, connector, _):
        super(AbstractSearchForObject, self).__init__(connector)
        self.run = 0
        self.pattern = connector.config["Head"]["SearchPattern"]
        self.look_at_old_position = connector.config["Head"]["Toggles"]["look_at_old_position"]

    def search(self, connector, point):
        rospy.logdebug('Searching...')
        self.run += 1
        u = point.x
        v = point.y

        if self.look_at_old_position and self.run <= 4:
            if self.run == 1 and not (u == 0.0 and v == 0.0) and u and v:
                # the object is not seen, so we first try to find it at its last position
                pan_tilt = connector.head.get_pantilt_from_uv(u, v)
                return self.push(HeadToPanTilt, pan_tilt)
            elif self.run == 2:
                # rechts vom Objekt suchen
                pan_tilt = connector.head.get_pantilt_from_uv(u, v)
                pan_tilt_right = pan_tilt[0] - connector.head.offset_right, pan_tilt[1]
                return self.push(HeadToPanTilt, pan_tilt_right)
            elif self.run == 3:
                # vor dem Objekt suchen
                pan_tilt = connector.head.get_pantilt_from_uv(u, v)
                pan_tilt_right = pan_tilt[0], pan_tilt[1] - connector.head.offset_down # TODO: make sure that 10° is enough
                return self.push(HeadToPanTilt, pan_tilt_right)
            elif self.run == 4:
                # links vom Objekt suchen
                pan_tilt = connector.head.get_pantilt_from_uv(u, v)
                pan_tilt_right = pan_tilt[0] + connector.head.offset_left, pan_tilt[1]
                return self.push(HeadToPanTilt, pan_tilt_right)
        else:
            # we try to find the object by using a pattern
            rospy.logdebug("Push: Continuous Search")
            return self.push(ContinuousSearch)


class SearchForBall(AbstractSearchForObject):
    def perform(self, connector, reevaluate=False):
        rospy.logdebug("Start Search for ball")
        ball = connector.personal_model.get_ball_relative_msg()
        return self.search(connector, ball)


class SearchForEnemyGoal(AbstractSearchForObject):
    def perform(self, connector, reevaluate=False):
        # Take any goal until we can distinguish between them
        goal = connector.personal_model.get_goal_relative()
        return self.search(connector, goal)
