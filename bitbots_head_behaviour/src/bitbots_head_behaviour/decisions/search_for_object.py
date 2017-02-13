# -*- coding:utf-8 -*-
"""
SearchForBall
^^^^^^^^^^^^^

Lets the head search only for the ball

History:

* 05.12.14: Created (Marc Bestmann)

"""

from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.head.actions.head_to_pan_tilt import HeadToPanTilt
from bitbots.modules.behaviour.head.decisions.continious_search import ContiniousSearch
from bitbots.modules.behaviour.head.helpers.head_helper import get_pantilt_from_uv
from bitbots.util.config import get_config


class AbstactSearchForObject(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):
        pass

    def __init__(self, _):
        super(AbstactSearchForObject, self).__init__()
        self.run = -1
        config = get_config()
        self.pattern = config["Behaviour"]["Common"]

    def search(self, connector, u, v):

        if not connector.raw_vision_capsule().is_new_frame():
            return

        self.run += 1

        if False and self.run <= 10 and not (u == 0 and v == 0):
            # the ball is not seen, so we first try to find it at its last position
            # (u, v) = connector.world_model_capsule().get_ball_position_uv()#todo mittelsweltmodell implentieren

            pan_tilt = get_pantilt_from_uv(u, v, connector.get_ipc())
            return self.push(HeadToPanTilt, pan_tilt)

        # elif self.run ==1: #todo do fancy stuff like looking left and right of the saved position
        else:
            # we try to find the ball by using a pattern
            return self.push(ContiniousSearch)


class SearchForBall(AbstactSearchForObject):
    def perform(self, connector, reevaluate=False):
        u, v = connector.filtered_vision_capsule().get_local_goal_model_ball()
        return self.search(connector, u, v)


class SearchForEnemyGoal(AbstactSearchForObject):
    def perform(self, connector, reevaluate=False):
        u, v = connector.filtered_vision_capsule().get_local_goal_model_opp_goal()
        return self.search(connector, u, v)
