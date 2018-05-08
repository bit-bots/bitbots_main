# -*- coding:utf-8 -*-
"""
SearchForBall
^^^^^^^^^^^^^

Lets the head search only for the ball

History:


"""
import rospy

from bitbots_head_behaviour.actions.look_at import LookAtRelativePoint
from bitbots_head_behaviour.decisions.continuous_search import ContinuousSearch
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class AbstractSearchForObject(AbstractDecisionModule):
    def __init__(self, connector, _):
        super(AbstractSearchForObject, self).__init__(connector)
        self.run = 0
        self.pattern = connector.config["Head"]["SearchPattern"]
        self.look_at_old_position = connector.config["Head"]["Toggles"]["look_at_old_position"]
        self.object_lost_time = connector.config["Head"]["Search"]["objectLostTime"]
        self.offset_right = connector.config["Head"]["Search"]["offsetRight"]
        self.offset_down = connector.config["Head"]["Search"]["offsetDown"]
        self.offset_left = connector.config["Head"]["Search"]["offsetLeft"]

    def perform(self, connector, reevaluate=False):
        pass

    def search(self, point, last_seen):
        rospy.logdebug('Searching...')
        self.run += 1
        u = point.x
        v = point.y

        if self.look_at_old_position and self.run <= 4 and rospy.get_time() - last_seen < self.object_lost_time:
            if self.run == 1:
                # the object is not seen, so we first try to find it at its last position
                look_at_point = (u, v, 0)
            elif self.run == 2:
                # Search to the right of the object
                look_at_point = (u + self.offset_right, v)
            elif self.run == 3:
                # Search in front of the object
                look_at_point = (u, v - self.offset_down)
            elif self.run == 4:
                # Search left of the object
                look_at_point = (u - self.offset_left, v)
            return self.push(LookAtRelativePoint, look_at_point)
        else:
            # we try to find the object by using a pattern
            rospy.logdebug("Push: Continuous Search")
            return self.push(ContinuousSearch)


class SearchForBall(AbstractSearchForObject):
    def perform(self, connector, reevaluate=False):
        rospy.logdebug("Start Search for ball")
        ball = connector.personal_model.get_ball_relative_msg()
        return self.search(ball, connector.personal_model.ball_last_seen())


class SearchForEnemyGoal(AbstractSearchForObject):
    def perform(self, connector, reevaluate=False):
        # Take any goal until we can distinguish between them
        goal = connector.personal_model.get_goal_relative()
        return self.search(goal, connector.personal_model.any_goal_last_seen())
