# -*- coding:utf-8 -*-
"""
SelfPositioningDecider
^^^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 11.05.14: Created (Sheepy)

This Module makes the decision for self positioning by pushing the SearchNearestGoal on the stack - this
module searches the nearest goal and transports the decision back here with a callback - after that
a specific walk pattern is applied and when this module here is not in reevaluate when performs called we
are finished with the walk pattern. This module gets reevaluated to jump out on state not ready anymore.
"""

from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.go_to_absolute_position import GoToAbsolutePosition
from bitbots.modules.keys import DATA_VALUE_STATE_READY
from bitbots.util import get_config

from bitbots_common.connector.connector import BodyConnector

config = get_config()


class SelfPositioningDecider(AbstractDecisionModule):
    def __init__(self, connector: BodyConnector, _):
        super(SelfPositioningDecider, self).__init__(connector)
        self.looked_which_goal_is_mine = False
        self.finished_positioning = False
        self.determination_result = None
        self.player = config["PLAYER"]
        position_ready_list_dict = {
            1: (-2000, 0, 0),
            2: (-2000, 1000, 0),
            3: (-2000, -1000, 0),
            4: (-1000, 1000, 0),
            5: (-1000, -1000, 0),
            6: (-200, 0, 0),
        }
        self.target_position = position_ready_list_dict[self.player]

    def callback_for_search_nearest_goal(self, val):
        """ We need a callback here because we need to transport a value """
        self.determination_result = val

    def perform(self, connector, reevaluate=False):

        print(connector.gamestatus_capsule().is_game_state_equals(DATA_VALUE_STATE_READY))

        if not connector.gamestatus_capsule().is_game_state_equals(DATA_VALUE_STATE_READY):
            return self.interrupt()

        # Decide if i am a goalie or not
        if connector.get_duty() == "Goalie":
            return self.push(GoToAbsolutePosition, (-4500, 0, 0))

        if connector.get_duty() == "Defender":
            return self.push(GoToAbsolutePosition, (-3000, 750, 0))

        return self.push(GoToAbsolutePosition, self.target_position)

    def get_reevaluate(self):
        return True

