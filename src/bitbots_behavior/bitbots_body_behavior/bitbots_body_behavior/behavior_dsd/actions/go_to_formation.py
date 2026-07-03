import math

import numpy as np
from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from tf2_geometry_msgs import PoseStamped
from ros2_numpy import numpify
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class GoToFormationPosition(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

        self.set_play = parameters.get("set_play", False)

        self.stand = parameters.get("stand", False)
        if self.stand:
            self.on_position_threshold = parameters["enter_position"]
            self.off_position_threshold = parameters["leave_position"]
            self.on_orientation_threshold = parameters["enter_orientation"]
            self.off_orientation_threshold = parameters["leave_orientation"]
        else:
            self.on_position_threshold = None
            self.off_position_threshold = None
            self.on_orientation_threshold = None
            self.off_orientation_threshold = None
        self.standing_on_target = False

    def perform(self, reevaluate=False):
        if self.set_play:
            optimal_positioning = self.blackboard.positioning.get_set_play_formation_assignment()
        else:
            optimal_positioning = self.blackboard.positioning.get_formation_assignment()

        own_target = optimal_positioning[self.blackboard.gamestate.get_own_id()]
        goal_pose = own_target["goal_pose"] # [x, y, orientation]

        current_pose = self.blackboard.world_model.get_current_position_pose_stamped()

        current_orientation = euler_from_quaternion(numpify(current_pose.pose.orientation))
        angle_to_goal_orientation = abs(math.remainder(current_orientation[2] - goal_pose[2], math.tau))


        distance = np.linalg.norm(goal_pose[:2] - numpify(current_pose.pose.position)[:2])

        self.publish_debug_data("current_orientation", current_orientation[2])
        self.publish_debug_data("goal_orientation", goal_pose[2])
        self.publish_debug_data("angle_to_goal_orientation", angle_to_goal_orientation)
        self.publish_debug_data("distance", distance)
        assert not self.stand or self.on_position_threshold < self.off_position_threshold, "on_position_threshold must be smaller than off_position_threshold"
        assert not self.stand or self.on_orientation_threshold < self.off_orientation_threshold, "on_orientation_threshold must be smaller than off_orientation_threshold"

        if self.stand and distance < self.on_position_threshold and angle_to_goal_orientation < self.on_orientation_threshold:
            self.standing_on_target = True

        if self.standing_on_target and (distance < self.off_position_threshold and angle_to_goal_orientation < self.off_orientation_threshold):
            # Cancel the path planning if it is running
            self.blackboard.pathfinding.cancel_goal()
            # need to keep publishing this since path planning publishes a few more messages
            self.blackboard.pathfinding.stop_walk()
        else:
            self.standing_on_target = False

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.blackboard.map_frame

            goal_pose = own_target["goal_pose"]
            pose_msg.pose.position.x = goal_pose[0]
            pose_msg.pose.position.y = goal_pose[1]
            quat = quaternion_from_euler(0, 0, goal_pose[2])
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            self.blackboard.pathfinding.publish(pose_msg)


