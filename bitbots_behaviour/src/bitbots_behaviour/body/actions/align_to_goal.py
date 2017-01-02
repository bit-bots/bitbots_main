# -*- coding:utf-8 -*-
"""
AlignToGoal
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 18.3.14: Created (Martin Poppinga)
Soll sich am ball in richtung des Tores ausrichten
The Robot repositionates so he is facing the opponent goal to score.
"""
import time

from bitbots.modules.abstract.abstract_action_module import AbstractActionModule
from bitbots.util import get_config

config = get_config()


class AlignToGoal(AbstractActionModule):
    def __init__(self, _):
        super(AlignToGoal, self).__init__()
        self.config_max_aligning_time = config["Behaviour"]["Fieldie"]["maxGoalAlignTime"]

    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().schedule_both_tracking()
        if not connector.blackboard_capsule().get_aligning_start_time():
            # First run, set the start time
            connector.blackboard_capsule().set_aligning_start_time()
        elif time.time() - connector.blackboard_capsule().get_aligning_start_time() \
                > self.config_max_aligning_time:
            # Give up aligning, i took too long
            connector.blackboard_capsule().stop_aligning()

        connector.walking_capsule().start_walking_plain(
            0,
            self.sign(connector.filtered_vision_capsule().get_local_goal_model_opp_goal()[1]) * -5,
            0)
