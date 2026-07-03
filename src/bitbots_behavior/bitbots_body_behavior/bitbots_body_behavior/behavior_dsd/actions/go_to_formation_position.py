import math

import numpy as np
from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from tf2_geometry_msgs import PoseStamped
from ros2_numpy import numpify
from tf_transformations import euler_from_quaternion


class GoToFormationPosition(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

        self.stand = parameters.get("stand", False)
        self.on_position_threshold = parameters["enter_position"]
        self.off_position_threshold = parameters["leave_position"]
        self.on_orientation_threshold = parameters["enter_orientation"]
        self.off_orientation_threshold = parameters["leave_orientation"]
        self.standing_on_target = False

    def perform(self, reevaluate=False):

        optimal_positioning = self.blackboard.positioning.get_formation_assignment()
        own_target = optimal_positioning[self.blackboard.gamestate.get_own_id()]

        current_pose = self.blackboard.world_model.get_current_position_pose_stamped()

        current_orientation = euler_from_quaternion(numpify(current_pose.pose.orientation))
        goal_orientation = euler_from_quaternion(numpify(pose.pose.orientation))
        angle_to_goal_orientation = abs(math.remainder(current_orientation[2] - goal_orientation[2], math.tau))


        distance = np.linalg.norm(numpify(pose.pose.position) - numpify(current_pose.pose.position))

        self.publish_debug_data("current_orientation", current_orientation[2])
        self.publish_debug_data("goal_orientation", goal_orientation[2])
        self.publish_debug_data("angle_to_goal_orientation", angle_to_goal_orientation)
        self.publish_debug_data("distance", distance)
        assert self.on_position_threshold < self.off_position_threshold, "on_position_threshold must be smaller than off_position_threshold"
        assert self.on_orientation_threshold < self.off_orientation_threshold, "on_orientation_threshold must be smaller than off_orientation_threshold"

        if distance < self.on_position_threshold and angle_to_goal_orientation < self.on_orientation_threshold and self.stand:
            self.standing_on_target = True

        if self.standing_on_target and (distance < self.off_position_threshold or angle_to_goal_orientation < self.off_orientation_threshold):
            # Cancel the path planning if it is running
            self.blackboard.pathfinding.cancel_goal()
            # need to keep publishing this since path planning publishes a few more messages
            self.blackboard.pathfinding.stop_walk()
        else:
            self.standing_on_target = False

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.blackboard.map_frame
                
            pose = own_target["goal_pose"]
            pose_msg.pose.position.x = pose[0]
            pose_msg.pose.position.y = pose[1]
            pose_msg.pose.orientation.w = pose[2]

            self.blackboard.pathfinding.publish(pose_msg)


