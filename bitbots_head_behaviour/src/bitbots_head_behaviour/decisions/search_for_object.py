"""
SearchForBall
^^^^^^^^^^^^^

Lets the head search only for the ball

History:


"""
import rospy

from bitbots_head_behaviour.actions.head_to_pan_tilt import HeadToPanTilt
from bitbots_head_behaviour.decisions.continious_search import ContiniousSearch
from bitbots_common.connector.connector import HeadConnector
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule


class AbstactSearchForObject(AbstractDecisionModule):
    def perform(self, connector: HeadConnector, reevaluate=False):
        pass

    def __init__(self, connector: HeadConnector, _):
        super(AbstactSearchForObject, self).__init__(connector)
        self.run = -1
        self.pattern = connector.config["Head"]["SearchPattern"]

    def search(self, connector: HeadConnector, u: float, v: float):

        self.run += 1

        if False and self.run <= 10 and not (u == 0.0 and v == 0.0):
            # the ball is not seen, so we first try to find it at its last position
            # (u, v) = connector.world_model_capsule().get_ball_position_uv()#todo mittelsweltmodell implentieren

            pan_tilt = get_pantilt_from_uv(u, v, connector.get_ipc())
            return self.push(HeadToPanTilt, pan_tilt)

        # elif self.run ==1: #todo do fancy stuff like looking left and right of the saved position
        else:
            # we try to find the ball by using a pattern
            rospy.logdebug("Push: Continious Search")
            return self.push(ContiniousSearch)


class SearchForBall(AbstactSearchForObject):
    def perform(self, connector: HeadConnector, reevaluate=False):
        rospy.logdebug("Start Searchfor ball")
        #u, v = connector.world_model.get_ballpos()
        u, v = None, None
        return self.search(connector, u, v)


class SearchForEnemyGoal(AbstactSearchForObject):
    def perform(self, connector: HeadConnector, reevaluate=False):
        u, v = connector.world_model.get_opp_goal_center_uv()
        return self.search(connector, u, v)
