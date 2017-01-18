"""
AlignToGoal
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>
The Robot repositionates so he is facing the opponent goal to score.
"""
import time

from stackmachine.abstract_action_module import AbstractActionModule

import rospy
from stackmachine.model import Connector


class AlignToGoal(AbstractActionModule):
    def __init__(self, _):
        super(AlignToGoal, self).__init__()
        self.config_max_aligning_time = rospy.get_param("/Behaviour/Fieldie/maxGoalAlignTime")

    def perform(self, connector: Connector, reevaluate=False):
        connector.blackboard.schedule_both_tracking()
        if not connector.blackboard.get_aligning_start_time():
            # First run, set the start time
            connector.blackboard.set_aligning_start_time()
        elif time.time() - connector.blackboard.get_aligning_start_time() \
                > self.config_max_aligning_time:
            # Give up aligning, i took too long
            connector.blackboard.stop_aligning()

        connector.walking.start_walking_plain(
            0,
            self.sign(connector.world_model.get_opp_goal_center_uv()[1]) * -5,
            0)
