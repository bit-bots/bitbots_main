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
import tf2_ros as tf2
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class AbstractLookAt(AbstractActionElement):
    def __init__(self, connector, args):
        super(AbstractLookAt, self).__init__(connector)
        self.tfBuffer = tf2.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2.TransformListener(self.tfBuffer)
        self.pan_speed_max = connector.config["Head"]["Search"]["maxPanSpeedSearch"]
        self.tilt_speed_max = connector.config["Head"]["Search"]["maxTiltSpeedSearch"]
        self.position_reached_time = rospy.get_time()

    def perform(self, connector, reevaluate=False):
        raise NotImplementedError

    def look_at(self, point, connector):
        """
        This method moves the head to a given point
        :param point: a PointStamped describing the point to look at
        """


        try:
            point_camera = self.tfBuffer.transform(point, 'head')
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            rospy.loginfo("Waiting for transform... ({})".format(e))
            return

        # Calculate the head joint angles and clip them to the right range
        angle_pan, angle_tilt = connector.head.get_motor_goals_from_point(point_camera.point)
        angle_pan = np.clip(np.rad2deg(angle_pan), connector.head.min_pan, connector.head.max_pan)
        angle_tilt = np.clip(np.rad2deg(angle_tilt), connector.head.min_tilt, connector.head.max_tilt)

        current_pan_pos, current_tilt_pos = connector.head.get_current_head_pos()
        if (abs(current_pan_pos - angle_pan) < connector.head.delta and
                abs(current_tilt_pos - angle_tilt) < connector.head.delta):
            # We reached the position
            if rospy.get_time() - self.position_reached_time > connector.head.wait_time:
                # We waited long enough, go back
                return self.pop()
            else:
                # Represent remaining wait time
                self.publish_debug_data("remaining_wait_time",connector.head.wait_time - (rospy.get_time() - self.position_reached_time))

        else:
            # We haven't reached it
            # Update when we should reach it
            self.position_reached_time = rospy.get_time()
            connector.head.send_motor_goals(angle_pan, 30.0, angle_tilt, 30.0)

            # Represent remaining tilt
            self.publish_debug_data("remaining_tilt",abs(current_pan_pos - angle_pan))
            self.publish_debug_data("remaining_pan",abs(current_tilt_pos - angle_tilt))


class LookAtBall(AbstractLookAt):
    """
    LookAtBall moves the head to look at the ball
    """
    def perform(self, connector, reevaluate=False):
        ball = connector.world_model.get_ball_stamped()
        self.look_at(ball, connector)


class LookAtGoal(AbstractLookAt):
    """
    LookAtGoal moves the head to look at the center of the goal
    """
    def perform(self, connector, reevaluate=False):
        goal_u, goal_v = connector.world_model.get_opp_goal_center_uv()
        goal = PointStamped()
        goal.header.frame_id = 'base_footprint'
        goal.point = Point(goal_u, goal_v, 0)
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
        point.header.frame_id = 'base_footprint'

        self.look_at(point, connector)

