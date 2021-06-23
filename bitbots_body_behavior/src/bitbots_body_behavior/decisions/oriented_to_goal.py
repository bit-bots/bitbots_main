import math

import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class OrientedToGoal(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Determines whether we are roughly looking towards the opponents goal.
        :param reevaluate:
        :return:
        """
        theta = self.blackboard.world_model.get_current_position()[2]
        if -math.tau / 4 < theta < math.tau / 4:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
