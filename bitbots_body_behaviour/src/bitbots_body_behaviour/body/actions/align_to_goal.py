# -*- coding:utf-8 -*-
"""
AlignToGoal
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>
The Robot repositionates so he is facing the opponent goal to score.
"""

import math
import rospy

from bitbots_body_behaviour.body.actions.go_to import GoToRelativePosition
from bitbots_stackmachine.abstract_action_module import AbstractActionModule
from humanoid_league_msgs.msg import HeadMode


class AlignToGoal(AbstractActionModule):
    def __init__(self, connector, _):
        super(AlignToGoal, self).__init__(connector)
        self.config_max_aligning_time = connector.config["Fieldie"]["maxGoalAlignTime"]

    def perform(self, connector, reevaluate=False):
        connector.blackboard.set_head_duty(HeadMode.BALL_GOAL_TRACKING)

        if not connector.blackboard.get_aligning_start_time():
            # First run, set the start time
            connector.blackboard.set_aligning_start_time()
        elif rospy.get_time() - connector.blackboard.get_aligning_start_time() \
                > self.config_max_aligning_time:
            # Give up aligning, i took too long
            connector.blackboard.stop_aligning()

        angle = math.atan2(connector.world_model.get_opp_goal_center_uv()[1],
                           connector.world_model.get_opp_goal_center_uv()[0])
        return self.push(GoToRelativePosition, (0, 0, angle))
