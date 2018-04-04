"""
AlignToGoal
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>
The Robot repositionates so he is facing the opponent goal to score.
"""
import time

from bitbots_stackmachine.abstract_action_module import AbstractActionModule

import rospy
from bitbots_common.connector.connector import BodyConnector


class AlignToGoal(AbstractActionModule):
    def __init__(self, connector: BodyConnector, _):
        super(AlignToGoal, self).__init__(connector)
        self.config_max_aligning_time = connector.config["Fieldie"]["maxGoalAlignTime"]

    def perform(self, connector: BodyConnector, reevaluate=False):
        connector.blackboard.set_head_duty("BALL_GOAL_TRACKING")

        if not connector.blackboard.get_aligning_start_time():
            # First run, set the start time
            connector.blackboard.set_aligning_start_time()
        elif rospy.get_time() - connector.blackboard.get_aligning_start_time() \
                > self.config_max_aligning_time:
            # Give up aligning, i took too long
            connector.blackboard.stop_aligning()

        connector.walking.start_walking_plain(
            0.04,
            self.sign(connector.world_model.get_opp_goal_center_uv()[1]) * -0.2,
            0)
