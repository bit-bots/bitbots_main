from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from tf.transformations import euler_from_quaternion
import math
import rospy


class AlignedToGoal(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(AlignedToGoal, self).__init__(blackboard, dsd, parameters)
        self.goalpost_safety_distance = self.blackboard.config['goalpost_safety_distance']
        self.field_length = rospy.get_param("field_length")
        self.goal_width = rospy.get_param("goal_width")

    def perform(self, reevaluate=False):
        """
        It is determined if the robot is correctly aligned to kick the ball into the goal.
        """
        current_pose = self.blackboard.world_model.get_current_position()
        current_ball = self.blackboard.world_model.get_ball_position_xy()
        if current_pose is None or current_ball is None:
            return 'NO'

        # calculate the distance of the intersection of a line from the robot throu the ball and the goal line.
        dist = abs(
            current_pose[1] +
            (current_pose[1] - current_ball[1]) *
            ((- current_pose[0] + self.field_length / 2) / (current_pose[0] - current_ball[0])))

        self.publish_debug_data("projected_distance_to_goal_center", dist)
        if dist < (self.goal_width / 2) - self.goalpost_safety_distance:
            return 'YES'
        else:
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

