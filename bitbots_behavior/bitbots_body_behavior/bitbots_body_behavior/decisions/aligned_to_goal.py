import math

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class AlignedToGoal(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.goalpost_safety_distance = self.blackboard.config["goalpost_safety_distance"]
        self.field_length = self.blackboard.world_model.field_length
        self.goal_width = self.blackboard.world_model.goal_width
        self.max_kick_angle = self.blackboard.config["max_kick_angle"]

    def perform(self, reevaluate=False):
        """
        It is determined if the robot is correctly aligned to kick the ball into the goal.
        Side kicks with maximum kick angle are counted as a possibilty to kick a goal.
        """
        # Get our position
        current_pose = self.blackboard.world_model.get_current_position()

        # Check if we know our position
        if current_pose is None:
            return "NO"

        # Get maximum kick angle relative to our base footprint
        angle_difference_right = current_pose[2] - self.max_kick_angle
        angle_difference_left = current_pose[2] + self.max_kick_angle

        # Calculate the intersection of the left most kick with the goal line right of the center of the goal. dist_left represents the y coordinate of the intersection.
        if math.cos(angle_difference_left) > 0:
            dist_left = current_pose[1] + (math.sin(angle_difference_left)) * (
                (-current_pose[0] + self.field_length / 2) / (math.cos(angle_difference_left))
            )
        else:
            # Set None if no intersection can be found in the desired direction
            dist_left = None

        # Do this also for the right most kick in a similar way
        if math.cos(angle_difference_right) > 0:
            dist_right = current_pose[1] + (math.sin(angle_difference_right)) * (
                (-current_pose[0] + self.field_length / 2) / (math.cos(angle_difference_right))
            )
        else:
            # Set None if no intersection can be found in the desired direction
            dist_right = None

        self.publish_debug_data("kick_max_left_goal_intersection", dist_left)
        self.publish_debug_data("kick_max_right_goal_intersection", dist_right)

        # Check if our left most kick is right of the right goal post
        if dist_left is not None and dist_left < -self.goal_width / 2 + self.goalpost_safety_distance:
            return "NO"

        # Check if our right most kick is left of the left most goal post
        if dist_right is not None and dist_right > self.goal_width / 2 - self.goalpost_safety_distance:
            return "NO"

        # Check if nether the left nor the right most kick intersect with the goal line. This happens if we are facing in e.g. the oposit direction
        if dist_left is None and dist_right is None:
            return "NO"

        # We are able to kick in the goal if all of this was not the case
        return "YES"

    def get_reevaluate(self):
        return True
