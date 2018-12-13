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
from geometry_msgs.msg import PoseStamped, Quaternion
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class GoToRelativePosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to a position relative to the robot
        :param dsd:

        """
        super(GoToRelativePosition, self).__init__(blackboard, dsd)
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(5.0))
        tf_listener = tf2.TransformListener(self.tf_buffer)
        self.point = parameters

    def perform(self, reevaluate=False):
        if not self.blackboard.config['use_move_base']:
            self.blackboard.pathfinding.pub_simple_pathfinding(self.point[0], self.point[1], self.point[2])
            return self.pop()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'base_link'

        pose_msg.pose.position.x = self.point[0]
        pose_msg.pose.position.y = self.point[1]
        pose_msg.pose.position.z = 0

        rotation = quaternion_from_euler(0, 0, math.radians(self.point[2]))
        pose_msg.pose.orientation = Quaternion(*rotation)

        try:
            absolute_pose = self.tf_buffer.transform(pose_msg, 'map', timeout=rospy.Duration(0.3))
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
            rospy.logdebug("Waiting for transform...")
            return

        # To have the object we are going to in front of us, go to a point behind it
        self.blackboard.pathfinding.call_action(absolute_pose)
        if self.blackboard.pathfinding.is_walking_active():
            return self.pop()


class GoToAbsolutePosition(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to an absolute position on the field
        :param dsd:

        """
        super(GoToAbsolutePosition, self).__init__(blackboard, dsd)
        self.point = parameters

    def perform(self, reevaluate=False):
        if not self.blackboard.pathfinding.useMoveBase:
            return self.pop()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = self.point[0]
        pose_msg.pose.position.y = self.point[1]
        pose_msg.pose.position.z = 0

        rotation = quaternion_from_euler(0, 0, math.radians(self.point[2]))
        pose_msg.pose.orientation = Quaternion(*rotation)

        self.blackboard.pathfinding.call_action(pose_msg)
        if self.blackboard.pathfinding.is_walking_active():
            return self.pop()



class GoToOwnGoal(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to the own goal
        :param dsd:

        """
        point = (blackboard.world_model.get_own_goal_center_xy()[0],
                 blackboard.world_model.get_own_goal_center_xy()[1],
                 parameters)
        super(GoToOwnGoal, self).__init__(blackboard, dsd, point)


class GoToEnemyGoal(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to the enemy goal
        :param dsd:

        """
        point = (blackboard.world_model.get_opp_goal_center_xy()[0],
                 blackboard.world_model.get_opp_goal_center_xy()[1],
                 parameters)
        super(GoToEnemyGoal, self).__init__(blackboard, dsd, point)


class GoToCenterpoint(GoToAbsolutePosition):
    def __init__(self, blackboard, dsd, parameters=None):
        """Go to the center of the field and look towards the enemy goal
        :param dsd:
        """
        point = 0, 0, 0
        super(GoToCenterpoint, self).__init__(blackboard, dsd, point)


class GoToDutyPosition(GoToAbsolutePosition):
    pass
