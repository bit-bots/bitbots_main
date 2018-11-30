# -*- coding:utf-8 -*-
"""
SearchForBall
^^^^^^^^^^^^^

Lets the head search only for the ball


"""
import rospy

from bitbots_head_behaviour.actions.look_at import LookAtRelativePoint
from bitbots_head_behaviour.decisions.continuous_search import ContinuousSearch
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement


class AbstractSearchForObject(AbstractDecisionElement):
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
        u = point[0]
        v = point[1]

        if self.look_at_old_position and self.run <= 4 and rospy.get_time() - last_seen < self.object_lost_time:
            self.publish_debug_data("mode","simple search")

            if self.run == 1:
                # the object is not seen, so we first try to find it at its last position
                self.publish_debug_data("strategy","look at last position")
                look_at_point = (u, v, 0)
            elif self.run == 2:
                # Search to the right of the object
                self.publish_debug_data("strategy","search to the right of last position")
                look_at_point = (u + self.offset_right, v, 0)
            elif self.run == 3:
                # Search in front of the object
                self.publish_debug_data("strategy","search in front of last position")
                look_at_point = (u, v - self.offset_down, 0)
            elif self.run == 4:
                # Search left of the object
                self.publish_debug_data("strategy","search to the left of last position")
                look_at_point = (u - self.offset_left, v, 0)

            return self.push(LookAtRelativePoint, look_at_point)

        else:
            # we try to find the object by using a pattern
            self.publish_debug_data("mode","continuous pattern search")
            rospy.logdebug("Push: Continuous Search")
            return self.push(ContinuousSearch)


class SearchForBall(AbstractSearchForObject):
    def perform(self, connector, reevaluate=False):
        rospy.logdebug("Start Search for ball")
        ball = connector.world_model.get_ball_position_uv()
        return self.search(ball, connector.world_model.ball_last_seen())


class SearchForEnemyGoal(AbstractSearchForObject):
    def perform(self, connector, reevaluate=False):
        # Take any goal until we can distinguish between them
        goal = connector.world_model.get_opp_goal_center_uv()
        return self.search(goal, connector.world_model.any_goal_last_seen())
