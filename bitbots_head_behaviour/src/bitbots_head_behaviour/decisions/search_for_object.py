"""
SearchForBall
^^^^^^^^^^^^^

Lets the head search only for the ball

History:


"""
import rospy
import math

from bitbots_head_behaviour.actions.head_to_pan_tilt import HeadToPanTilt
from bitbots_head_behaviour.decisions.continious_search import ContiniousSearch
from bitbots_common.connector.connector import HeadConnector
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class AbstactSearchForObject(AbstractDecisionModule):
    def perform(self, connector: HeadConnector, reevaluate=False):
        pass

    def __init__(self, connector: HeadConnector, _):
        super(AbstactSearchForObject, self).__init__(connector)
        self.run = 0
        self.pattern = connector.config["Head"]["SearchPattern"]

    def search(self, connector: HeadConnector):
        self.run += 1
        rospy.loginfo(self.run)
        u, v = connector.world_model.get_ball_position_uv()

        if self.run == 1 and not (u == 0.0 and v == 0.0) and u and v:
            # the ball is not seen, so we first try to find it at its last position
            pan_tilt = self.get_pantilt_from_uv(u, v, connector)
            rospy.loginfo('Pushed pan {0} and tilt {1}'.format(u, v))
            return self.push(HeadToPanTilt, pan_tilt)
        elif self.run == 2:
            # rechts vom Ball suchen
            pan_tilt = self.get_pantilt_from_uv(u, v, connector)
            pan_tilt_right = pan_tilt[0] + connector.head.offset_right, pan_tilt[1] # TODO: make sure that right is + and left is -
            return self.push(HeadToPanTilt, pan_tilt_right)
        elif self.run == 3:
            # vor dem Ball suchen
            pan_tilt = self.get_pantilt_from_uv(u, v, connector)
            pan_tilt_right = pan_tilt[0], pan_tilt[1] - connector.head.offset_down # TODO: make sure that 10° is enough
            return self.push(HeadToPanTilt, pan_tilt_right)
        elif self.run == 4:
            # links vom Ball suchen
            pan_tilt = self.get_pantilt_from_uv(u, v, connector)
            pan_tilt_right = pan_tilt[0] - connector.head.offset_left, pan_tilt[1] # TODO: make sure that right is + and left is -
            return self.push(HeadToPanTilt, pan_tilt_right)
        else:
            # we try to find the ball by using a pattern
            rospy.loginfo("Push: Continious Search")
            return self.push(ContiniousSearch)

    def get_pantilt_from_uv(self, u, v, connector):
        # type: (float, float, HeadConnector) -> tuple
        cam_height = connector.head.camera_height
        ball_height = connector.head.ball_height
        if u == 0.0:
            pan = 90
        else:
            pan = math.degrees(math.atan(v/u))

        tilt = -math.degrees(math.atan((cam_height - ball_height / 2)/(math.sqrt(u ** 2 + v ** 2))))
        return pan, tilt

class SearchForBall(AbstactSearchForObject):
    def perform(self, connector: HeadConnector, reevaluate=False):
        rospy.logdebug("Start Search for ball")
        return self.search(connector)


class SearchForEnemyGoal(AbstactSearchForObject):
    def perform(self, connector: HeadConnector, reevaluate=False):
        u, v = connector.world_model.get_opp_goal_center_uv()
        return self.search(connector, u, v)
