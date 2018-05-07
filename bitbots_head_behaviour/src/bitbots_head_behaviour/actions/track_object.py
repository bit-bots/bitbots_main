# -*- coding:utf-8 -*-
"""
ConfirmGoal
^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

"""
import rospy

from bitbots_stackmachine.abstract_init_action_module import AbstractInitActionModule
from bitbots_head_behaviour.actions.head_to_pan_tilt import HeadToPanTilt

class AbstractTrackObject(AbstractInitActionModule):
    """
    Confirmed either the Ball, OwnGoal or EnemyGoal by passing it to the init arg
    """
    def __init__(self, connector, args=None):
        super(AbstractTrackObject, self).__init__(connector, args)

        # this influences how precise the ball has to be in the center to make the head move
        self.a_sens = connector.config["Head"]["Tracking"]["xSensivity"]
        self.b_sens = connector.config["Head"]["Tracking"]["ySensivity"]
        self.b_center_default = connector.config["Head"]["Tracking"]["yCenterDefault"]
        self.b_center_goalie = connector.config["Head"]["Tracking"]["yCenterGoalie"]

        # just the max and min angles for moving the head
        self.max_tilt = connector.config["Head"]["Camera"]["maxTilt"]
        self.min_tilt = connector.config["Head"]["Camera"]["minTilt"]
        self.min_pan = connector.config["Head"]["Camera"]["minPan"]
        self.max_pan = connector.config["Head"]["Camera"]["maxPan"]

        self.max_tilt_speed = connector.config["Head"]["Tracking"]["maxTiltSpeedTracking"]
        self.max_pan_speed = connector.config["Head"]["Tracking"]["maxPanSpeedTracking"]

        # propably camera angle
        self.angle = connector.config["Head"]["Camera"]["cameraAngle"]

        # to compute the camera angle in vertical (16:9)
        self.horizontal_factor = connector.config["Head"]["Camera"]["horizontalFactor"]
        self.vertical_factor = connector.config["Head"]["Camera"]["verticalFactor"]

    def track_with_values(self, connector, u, v):
        rospy.logdebug('Tracking...')
        pan_tilt = connector.head.get_pantilt_from_uv(u, v)
        return self.push(HeadToPanTilt, pan_tilt)

    def perform(self, connector, reevaluate=None):
        raise NotImplementedError


class TrackBall(AbstractTrackObject):
    def perform(self, connector, reevaluate=False):
        u, v = connector.world_model.get_ball_position_uv()
        self.track_with_values(connector, u, v)


class TrackGoal(AbstractTrackObject):
    def perform(self, connector, reevaluate=None):
        # TODO: Distinguish between own and enemy goal (get data from world model)
        a, b = connector.personal_model.get_goal_relative()
        self.track_with_values(connector, a, b)
