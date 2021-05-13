import numpy as np

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class ReachedMovebaseGoalPosition(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(ReachedMovebaseGoalPosition, self).__init__(blackboard, dsd, parameters)

        self.threshould = parameters['thres']

    def perform(self, reevaluate=False):
        """
        Determines whether we are near the movebase goal
        :param reevaluate:
        :return:
        """

        if self.blackboard.pathfinding.goal is None or self.blackboard.pathfinding.current_pose is None:
            return "NO"

        goal = np.array([self.blackboard.pathfinding.goal.pose.position.x, self.blackboard.pathfinding.goal.pose.position.y])
        position = np.array([self.blackboard.pathfinding.current_pose.pose.position.x, self.blackboard.pathfinding.current_pose.pose.position.y])
        if np.linalg.norm(goal - position) < self.threshould:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
