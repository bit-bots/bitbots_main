# -*- encoding:utf-8 -*-
"""
GoTo
^^^^

.. moduleauthor: Timon Engelke <7engelke@informatik.uni-hamburg.de>

Goes to a position or an object
"""

import math
import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion
from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class GoToRelativePosition(AbstractActionModule):
    def __init__(self, connector, args):
        """Go to a position relative to the robot

        :param args: a list consisting of u, v and theta where u is the distance
                     to the front, v is distance to the left and theta is the angle
                     of rotation in degrees
        """
        super(GoToRelativePosition, self).__init__(connector)
        self.point = args

    def perform(self, connector, reevaluate=False):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'base_link'

        pose_msg.pose.position.x = self.point[0]
        pose_msg.pose.position.y = self.point[1]
        pose_msg.pose.position.z = 0

        rotation = quaternion_from_euler(0, 0, math.radians(self.point[2]))
        pose_msg.pose.orientation = Quaternion(*rotation)

        connector.pathfinding_publisher.publish(pose_msg)
        return self.pop()


class GoToAbsolutePosition(AbstractActionModule):
    def __init__(self, connector, args):
        """Go to an absolute position on the field

        :param args: a list consisting of x, y and theta where x is the distance
                     from the center point to the front, y is distance to the left
                     and theta is the angle of rotation from the current position
                     in degrees
        """
        super(GoToAbsolutePosition, self).__init__(connector)
        self.point = args

    def perform(self, connector, reevaluate=False):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = self.point[0]
        pose_msg.pose.position.y = self.point[1]
        pose_msg.pose.position.z = 0

        rotation = quaternion_from_euler(0, 0, math.radians(self.point[2]))
        pose_msg.pose.orientation = Quaternion(*rotation)

        connector.pathfinding_publisher.publish(pose_msg)
        self.pop()


class GoToBall(GoToRelativePosition):
    def __init__(self, connector, args=0):
        """Go to the ball

        :param args: the angle of relative rotation in degrees
        """
        point = (connector.personal_model.get_ball_relative()[0],
                 connector.personal_model.get_ball_relative()[1],
                 args)
        super(GoToBall, self).__init__(connector, point)


class GoToOwnGoal(GoToAbsolutePosition):
    def __init__(self, connector, args):
        """Go to the own goal

        :param args: the angle of absolute rotation in degrees
        """
        point = (connector.world_model.get_own_goal_center_xy()[0],
                 connector.world_model.get_own_goal_center_xy()[1],
                 args)
        super(GoToOwnGoal, self).__init__(connector, point)


class GoToEnemyGoal(GoToAbsolutePosition):
    def __init__(self, connector, args):
        """Go to the enemy goal

        :param args: the angle of absolute rotation in degrees
        """
        point = (connector.world_model.get_opp_goal_center_xy()[0],
                 connector.world_model.get_opp_goal_center_xy()[1],
                 args)
        super(GoToEnemyGoal, self).__init__(connector, point)


class GoToCenterpoint(GoToAbsolutePosition):
    def __init__(self, connector):
        """Go to the center of the field and look towards the enemy goal"""
        point = 0, 0, 0
        super(GoToCenterpoint, self).__init__(connector, point)
