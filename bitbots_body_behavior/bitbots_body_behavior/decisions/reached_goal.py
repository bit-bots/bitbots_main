import math

import numpy as np
from bitbots_blackboard.blackboard import BodyBlackboard
from tf_transformations import euler_from_quaternion

from dynamic_stack_decider.abstract_decision_element import \
    AbstractDecisionElement


class ReachedPathPlanningGoalPosition(AbstractDecisionElement):
    blackboard: BodyBlackboard
    def __init__(self, blackboard, dsd, parameters=None):
        super(ReachedPathPlanningGoalPosition, self).__init__(blackboard, dsd, parameters)

        self.threshould = parameters['thres']

    def perform(self, reevaluate=False):
        """
        Determines whether we are near the path planning goal
        :param reevaluate:
        :return:
        """

        current_pose = self.blackboard.world_model.get_current_position_pose_stamped()
        goal_pose = self.blackboard.pathfinding.get_goal()

        if current_pose is None or goal_pose is None:
            return "NO"

        goal = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y])
        position = np.array([current_pose.pose.position.x, current_pose.pose.position.y])
        distance = np.linalg.norm(goal - position)
        self.publish_debug_data("distance", distance)
        if distance < self.threshould:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True


class AlignedToPathPlanningGoal(AbstractDecisionElement):
    blackboard: BodyBlackboard
    def __init__(self, blackboard, dsd, parameters=None):
        super(AlignedToPathPlanningGoal, self).__init__(blackboard, dsd, parameters)
        self.orientation_threshold = self.blackboard.config['goal_alignment_orientation_threshold']  # [deg]

    def perform(self, reevaluate=False):
        """
        It is determined if the robot is correctly aligned to the orientation of the path planning goal within a
        determined threshold by comparing the current orientation angle of the robot in the map with the one from the
        path planning goal.
        """
        current_pose = self.blackboard.world_model.get_current_position_pose_stamped()
        current_goal = self.blackboard.pathfinding.get_goal()
        if current_pose is None or current_goal is None:
            # When the path planning did not received a goal yet, no current position on the map is known.
            # If it is not known if the robot is aligned correctly to, e.g., the goal the robot
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