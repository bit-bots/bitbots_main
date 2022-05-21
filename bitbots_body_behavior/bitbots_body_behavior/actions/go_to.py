# -*- encoding:utf-8 -*-
"""
GoTo
^^^^

.. moduleauthor: Timon Engelke <7engelke@informatik.uni-hamburg.de>

Goes to a position or an object
"""

import math
import rospy
import tf2_ros as tf2
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Quaternion
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class GoToRelativePosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to a position relative to the robot
        :param dsd:

        """
        super(GoToRelativePosition, self).__init__(blackboard, dsd)
        self.point = float(parameters.get('x', 0)), float(parameters.get('y', 0)), float(parameters.get('t', 0))
        self.first = True

    def perform(self, reevaluate=False):
        if self.first:
            self.first = False
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = self.blackboard.base_footprint_frame

            pose_msg.pose.position.x = self.point[0]
            pose_msg.pose.position.y = self.point[1]
            pose_msg.pose.position.z = 0

            rotation = quaternion_from_euler(0, 0, math.radians(self.point[2]))
            pose_msg.pose.orientation = Quaternion(*rotation)

            # To have the object we are going to in front of us, go to a point behind it
            self.blackboard.pathfinding.publish(pose_msg)
            # TODO: this in good
            # waiting until the robot started to walk
            rospy.sleep(0.25)
        if not self.blackboard.blackboard.is_currently_walking():
            self.pop()


class GoToAbsolutePosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to an absolute position on the field
        :param dsd:

        """
        super(GoToAbsolutePosition, self).__init__(blackboard, dsd)
        self.point = parameters

    def perform(self, reevaluate=False):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.blackboard.map_frame

        pose_msg.pose.position.x = self.point[0]
        pose_msg.pose.position.y = self.point[1]
        pose_msg.pose.position.z = 0

        rotation = quaternion_from_euler(0, 0, math.radians(self.point[2]))
        pose_msg.pose.orientation = Quaternion(*rotation)

        self.blackboard.pathfinding.publish(pose_msg)


class GoToOwnGoal(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to the own goal
        :param dsd:

        """
        point = (blackboard.world_model.get_map_based_own_goal_center_xy()[0],
                 blackboard.world_model.get_map_based_own_goal_center_xy()[1],
                 parameters)
        super(GoToOwnGoal, self).__init__(blackboard, dsd, point)


class GoToEnemyGoal(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to the enemy goal
        :param dsd:

        """
        point = (blackboard.world_model.get_map_based_opp_goal_center_xy()[0],
                 blackboard.world_model.get_map_based_opp_goal_center_xy()[1],
                 parameters)
        super(GoToEnemyGoal, self).__init__(blackboard, dsd, point)


class GoToCenterpoint(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to the center of the field and look towards the enemy goal
        :param dsd:
        """
        point = 0, 0, 0
        super(GoToCenterpoint, self).__init__(blackboard, dsd, point)

