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
from bitbots_common.connector.connector import HeadConnector
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class AbstractSearchForObject(AbstractDecisionModule):
    def perform(self, connector: HeadConnector, reevaluate=False):
        pass

    def __init__(self, connector: HeadConnector, _):
        super(AbstractSearchForObject, self).__init__(connector)
        self.run = 0
        self.pattern = connector.config["Head"]["SearchPattern"]

    def search(self, connector: HeadConnector):
        rospy.logdebug('Searching...')
        self.run += 1
        u, v = connector.world_model.get_ball_position_uv()

        # TODO: use config look_at_old_position
        if self.run == 1 and not (u == 0.0 and v == 0.0) and u and v:
            # the ball is not seen, so we first try to find it at its last position
            pan_tilt = connector.head.get_pantilt_from_uv(u, v)
            return self.push(HeadToPanTilt, pan_tilt)
        elif self.run == 2:
            # rechts vom Ball suchen
            pan_tilt = connector.head.get_pantilt_from_uv(u, v)
            pan_tilt_right = pan_tilt[0] + connector.head.offset_right, pan_tilt[1] # TODO: make sure that right is + and left is -
            return self.push(HeadToPanTilt, pan_tilt_right)
        elif self.run == 3:
            # vor dem Ball suchen
            pan_tilt = connector.head.get_pantilt_from_uv(u, v)
            pan_tilt_right = pan_tilt[0], pan_tilt[1] - connector.head.offset_down # TODO: make sure that 10° is enough
            return self.push(HeadToPanTilt, pan_tilt_right)
        elif self.run == 4:
            # links vom Ball suchen
            pan_tilt = connector.head.get_pantilt_from_uv(u, v)
            pan_tilt_right = pan_tilt[0] - connector.head.offset_left, pan_tilt[1] # TODO: make sure that right is + and left is -
            return self.push(HeadToPanTilt, pan_tilt_right)
        else:
            # we try to find the ball by using a pattern
            rospy.logdebug("Push: Continuous Search")
            return self.push(ContinuousSearch)


class SearchForBall(AbstractSearchForObject):
    def perform(self, connector: HeadConnector, reevaluate=False):
        rospy.logdebug("Start Search for ball")
        return self.search(connector)


class SearchForEnemyGoal(AbstractSearchForObject):
    def perform(self, connector: HeadConnector, reevaluate=False):
        u, v = connector.world_model.get_opp_goal_center_uv()
        return self.search(connector, u, v)
