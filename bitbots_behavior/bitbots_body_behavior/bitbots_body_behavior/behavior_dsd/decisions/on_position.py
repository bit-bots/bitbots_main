import math

import numpy as np
from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from ros2_numpy import numpify
from tf_transformations import euler_from_quaternion

from bitbots_body_behavior.behavior_dsd.actions.go_to_block_position import GoToBlockPosition


class OnBlockPosition(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.go_to_block_postion_action = GoToBlockPosition(blackboard, dsd, parameters)
        self.threshold = parameters.get("threshold", 0.0)
        self.orientation_threshold = math.radians(self.blackboard.config["goal_alignment_orientation_threshold"])

    def perform(self, reevaluate=False):
        """
        Determines whether we are on the block position as goalie
        """
        self.go_to_block_postion_action.perform()
        current_pose = self.blackboard.world_model.get_current_position_pose_stamped()
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
            return "YES"
        return "NO"
