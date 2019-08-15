from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from tf.transformations import euler_from_quaternion
import math


class AlignedToGoal(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(AlignedToGoal, self).__init__(blackboard, dsd, parameters)
        self.orientation_threshold = 10.0  # [deg]

    def perform(self, reevaluate=False):
        """ It is determined if the robot is correctly aligned to the orientation of the move_base goal within a
        determined threshold by comparing the current orientation angle of the robot in the map with the one from the
        move_base goal."""
        if self.blackboard.pathfinding.get_current_pose() is None:
            # When move_base did not received a goal yet, no current position on the map is known.
            # In this case it is not know if the robot is aligned correctly to, e.g., the goal and therefore the robot
            # should not be allowed to kick the ball.
            return 'NO'
        current_orientation = self.blackboard.pathfinding.get_current_pose().pose.orientation
        current_orientation = euler_from_quaternion([current_orientation.x, current_orientation.y,
                                                     current_orientation.z, current_orientation.w])
        goal_orientation = self.blackboard.pathfinding.get_goal().pose.orientation
        goal_orientation = euler_from_quaternion([goal_orientation.x, goal_orientation.y, goal_orientation.z,
                                                  goal_orientation.w])
        if math.degrees(abs(current_orientation[2] - goal_orientation[2])) < self.orientation_threshold:
            return 'YES'
        else:
            return 'NO'

    def get_reevaluate(self):
        return True
