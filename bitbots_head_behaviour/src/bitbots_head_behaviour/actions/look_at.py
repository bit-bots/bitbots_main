# -*- coding:utf-8 -*-
"""
LookAt
^^^^^^

.. moduleauthor:: Timon Engelke <7engelke@informatik.uni-hamburg.de>

This action moves the head to look at a given position on the floor relative to the robot.

The module expects a tuple containing the relative coordinates.
"""

import numpy as np
import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped

from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class AbstractLookAt(AbstractActionModule):
    def __init__(self, connector, args):
        super(AbstractLookAt, self).__init__(connector)
        self.tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def perform(self, connector, reevaluate=False):
        raise NotImplementedError

    def look_at(self, point, connector):
        """
        This method moves the head to a given point
        :param point: a PointStamped describing the point to look at
        """
        try:
            point_camera = self.tfBuffer.transform(point, 'camera_fixed')
        except tf2_ros.LookupException:
            print("Waiting for transform...")
            return

        angle_tilt = np.rad2deg(np.arctan2(point_camera.point.z, point_camera.point.x))
        angle_pan = np.rad2deg(np.arctan2(point_camera.point.y, point_camera.point.x))

        connector.head.send_motor_goals(angle_pan, 60, angle_tilt, 60)


class LookAtBall(AbstractLookAt):
    """
    LookAtBall moves the head to look at the ball
    """
    def perform(self, connector, reevaluate=False):
        ball = connector.personal_model.get_ball_relative_stamped()
        self.look_at(ball, connector)


class LookAtGoal(AbstractLookAt):
    """
    LookAtGoal moves the head to look at the center of the goal
    """
    def perform(self, connector, reevaluate=False):
        goal = connector.personal_model.get_goal_relative_stamped()
        self.look_at(goal, connector)


class LookAtRelativePoint(AbstractLookAt):
    """
    LookAtRelativePoint moves the head to look at a given point.
    It expects a tuple of three values describing a point relative
    to the feet of the robot. The tuple consists of the values u, v and w
    referring to distance to the front, to the left and height
    respectively.
    """
    def __init__(self, connector, args, init_data=None):
        super(LookAtRelativePoint, self).__init__(connector, init_data)
        self.u = args[0]
        self.v = args[1]
        self.w = args[2]

    def perform(self, connector, reevaluate=False):
        point = PointStamped()
        point.point.x = self.u
        point.point.y = self.v
        point.point.z = self.w
        # TODO: use base_foot_print
        point.header.frame_id = 'r_sole'

        self.look_at(point, connector)

