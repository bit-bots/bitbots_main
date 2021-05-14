import math
import numpy as np
from tf.transformations import euler_from_quaternion

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


class AlignedToMoveBaseGoal(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(AlignedToMoveBaseGoal, self).__init__(blackboard, dsd, parameters)
        self.orientation_threshold = self.blackboard.config['goal_alignment_orientation_threshold']  # [deg]

    def perform(self, reevaluate=False):
        """
        It is determined if the robot is correctly aligned to the orientation of the move_base goal within a
        determined threshold by comparing the current orientation angle of the robot in the map with the one from the
        move_base goal.
        """
        current_pose = self.blackboard.pathfinding.get_current_pose()
        current_goal = self.blackboard.pathfinding.get_goal()
        if current_pose is None or current_goal is None:
            # When move_base did not received a goal yet, no current position on the map is known.
            # In this case it is not know if the robot is aligned correctly to, e.g., the goal and therefore the robot
            # should not be allowed to kick the ball.
            return 'NO'
        current_orientation = current_pose.pose.orientation
        current_orientation = euler_from_quaternion([current_orientation.x, current_orientation.y,
                                                     current_orientation.z, current_orientation.w])
        goal_orientation = current_goal.pose.orientation
        goal_orientation = euler_from_quaternion([goal_orientation.x, goal_orientation.y, goal_orientation.z,
                                                  goal_orientation.w])
        if math.degrees(abs(current_orientation[2] - goal_orientation[2])) < self.orientation_threshold:
            return 'YES'
        else:
            return 'NO'

    def get_reevaluate(self):
        return True