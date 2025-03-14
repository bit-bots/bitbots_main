import math
from typing import Optional

import numpy as np
import tf2_ros as tf2
from bitbots_blackboard.body_blackboard import BodyBlackboard
from bitbots_utils.transforms import quat_from_yaw
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration


class GoToRelativePosition(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.point = float(parameters.get("x", 0)), float(parameters.get("y", 0)), float(parameters.get("t", 0))
        self.threshold = float(parameters.get("threshold", 0.1))
        self.first = True
        self.goal_pose: Optional[PoseStamped] = None

    def perform(self, reevaluate=False):
        if self.first:
            self.first = False
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.blackboard.node.get_clock().now().to_msg()
            goal_pose.header.frame_id = self.blackboard.base_footprint_frame

            goal_pose.pose.position.x = self.point[0]
            goal_pose.pose.position.y = self.point[1]
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation = quat_from_yaw(math.radians(self.point[2]))

            try:
                self.goal_pose = self.blackboard.tf_buffer.transform(
                    goal_pose, self.blackboard.map_frame, timeout=Duration(seconds=0.5)
                )
                self.blackboard.pathfinding.publish(self.goal_pose)
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                self.blackboard.node.get_logger().warning("Could not transform goal pose: " + str(e))
                self.first = False
        else:
            current_position = self.blackboard.world_model.get_current_position()
            if self.goal_pose is not None and current_position is not None:
                position = np.array(current_position[:2])
                goal = np.array([self.goal_pose.pose.position.x, self.goal_pose.pose.position.y])
                distance = np.linalg.norm(goal - position)
                if distance < self.threshold:
                    self.pop()


class GoToAbsolutePosition(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        """Go to an absolute position on the field"""
        super().__init__(blackboard, dsd, parameters)
        self.point = float(parameters.get("x", 0)), float(parameters.get("y", 0)), float(parameters.get("t", 0))
        self.blocking = parameters.get("blocking", True)

    def perform(self, reevaluate=False):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.blackboard.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.blackboard.map_frame

        pose_msg.pose.position.x = self.point[0]
        pose_msg.pose.position.y = self.point[1]
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = quat_from_yaw(math.radians(self.point[2]))

        self.blackboard.pathfinding.publish(pose_msg)

        if not self.blocking:
            self.pop()


class GoToAbsolutePositionFieldFraction(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters):
        """Go to an absolute position of the field, specified by the fraction of the field size"""
        super().__init__(blackboard, dsd, parameters)
        point = float(parameters.get("x", 0)), float(parameters.get("y", 0)), float(parameters.get("t", 0))
        self.point = (
            point[0] * self.blackboard.world_model.field_length / 2,
            point[1] * self.blackboard.world_model.field_width / 2,
            self.point[2],
        )


class GoToOwnGoal(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters):
        """Go to the own goal"""
        super().__init__(blackboard, dsd, parameters)
        self.point = (
            self.blackboard.world_model.get_map_based_own_goal_center_xy()[0],
            self.blackboard.world_model.get_map_based_own_goal_center_xy()[1],
            self.point[2],
        )


class GoToEnemyGoal(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters):
        """Go to the enemy goal"""
        super().__init__(blackboard, dsd, parameters)
        self.point = (
            self.blackboard.world_model.get_map_based_opp_goal_center_xy()[0],
            self.blackboard.world_model.get_map_based_opp_goal_center_xy()[1],
            self.point[2],
        )


class GoToCenterpoint(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters):
        """Go to the center of the field and look towards the enemy goal"""
        super().__init__(blackboard, dsd, parameters)
        self.point = 0.0, 0.0, 0.0
