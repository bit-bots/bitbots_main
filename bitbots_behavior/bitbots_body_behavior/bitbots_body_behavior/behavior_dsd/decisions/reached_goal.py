import math

import numpy as np
from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from ros2_numpy import numpify
from tf_transformations import euler_from_quaternion


class ReachedPathPlanningGoalPosition(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.frame_id = parameters.get("frame_id", self.blackboard.map_frame)
        self.threshold = parameters.get("threshold")
        self.latch = parameters.get("latch", False)
        self.latched = False

    def perform(self, reevaluate=False):
        """
        Determines whether we are near the path planning goal
        :param reevaluate:
        :return:
        """
        # We return that reached it forever if we reached it once if latching is enabled
        if self.latched:
            return "YES"

        current_pose = self.blackboard.world_model.get_current_position_pose_stamped(self.frame_id)
        goal_pose = self.blackboard.pathfinding.get_goal()

        if current_pose is None or goal_pose is None:
            return "NO"

        distance = np.linalg.norm(numpify(goal_pose.pose.position) - numpify(current_pose.pose.position))
        self.publish_debug_data("distance", distance)
        if distance < self.threshold:
            self.latched = self.latch  # Set it to true if we always want to return YES in the future
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class AlignedToPathPlanningGoal(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.orientation_threshold = math.radians(self.blackboard.config["goal_alignment_orientation_threshold"])
        self.frame_id = parameters.get("frame_id", self.blackboard.map_frame)

    def perform(self, reevaluate=False):
        """
        It is determined if the robot is correctly aligned to the orientation of the path planning goal within a
        determined threshold by comparing the current orientation angle of the robot in the map with the one from the
        path planning goal.
        """
        current_pose = self.blackboard.world_model.get_current_position_pose_stamped(self.frame_id)
        current_goal = self.blackboard.pathfinding.get_goal()

        if current_pose is None or current_goal is None:
            # When the path planning did not received a goal yet, no current position on the map is known.
            # If it is not known if the robot is aligned correctly to, e.g., the goal the robot
            # should not be allowed to kick the ball.
            return "NO"
        current_orientation = euler_from_quaternion(numpify(current_pose.pose.orientation))
        goal_orientation = euler_from_quaternion(numpify(current_goal.pose.orientation))
        angle_to_goal_orientation = abs(math.remainder(current_orientation[2] - goal_orientation[2], math.tau))

        if angle_to_goal_orientation < self.orientation_threshold:
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True


class ReachedAndAlignedToPathPlanningGoalPosition(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.frame_id = parameters.get("frame_id", self.blackboard.map_frame)
        self.threshold = parameters.get("threshold")
        self.orientation_threshold = math.radians(self.blackboard.config["goal_alignment_orientation_threshold"])
        self.frame_id = parameters.get("frame_id", self.blackboard.map_frame)
        self.latch = parameters.get("latch", False)
        self.latched = False

    def perform(self, reevaluate=False):
        """
        Determines whether we are near the path planning goal
        :param reevaluate:
        :return:
        """
        # We return that reached it forever if we reached it once if latching is enabled
        if self.latched:
            return "YES"

        current_pose = self.blackboard.world_model.get_current_position_pose_stamped(self.frame_id)
        goal_pose = self.blackboard.pathfinding.get_goal()

        if current_pose is None or goal_pose is None:
            return "NO"

        current_orientation = euler_from_quaternion(numpify(current_pose.pose.orientation))
        goal_orientation = euler_from_quaternion(numpify(goal_pose.pose.orientation))
        angle_to_goal_orientation = abs(math.remainder(current_orientation[2] - goal_orientation[2], math.tau))
        self.publish_debug_data("current_orientation", current_orientation[2])
        self.publish_debug_data("goal_orientation", goal_orientation[2])
        self.publish_debug_data("angle_to_goal_orientation", angle_to_goal_orientation)

        distance = np.linalg.norm(numpify(goal_pose.pose.position) - numpify(current_pose.pose.position))
        self.publish_debug_data("distance", distance)
        if distance < self.threshold and angle_to_goal_orientation < self.orientation_threshold:
            self.latched = self.latch  # Set it to true if we always want to return YES in the future
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
