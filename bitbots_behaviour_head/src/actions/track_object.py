# -*- coding:utf-8 -*-
"""
ConfirmGoal
^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:

* 06.03.14: Created (Marc)
* 10-12.03.14: Changed to use config values (Marc)

"""

from bitbots.modules.abstract.abstract_init_action_module import AbstractInitActionModule
from bitbots.util import get_config


class AbstactTrackObject(AbstractInitActionModule):
    """
    Confirmed either the Ball, OwnGoal or EnemyGoal by passing it to the init arg
    """

    def __init__(self, args=None):
        super(AbstactTrackObject, self).__init__(args)

        ###
        # ##load the part of the Config that is necessary
        ###
        # self.behaviour_actions_config = self.config["Behaviour"]["HeadActions"]
        self.common_behaviour_config = get_config()["Behaviour"]["Common"]

        # this influences how precise the ball has to be in the center to make the head move
        self.a_sens = self.common_behaviour_config["Tracking"]["xSensivity"]
        self.b_sens = self.common_behaviour_config["Tracking"]["ySensivity"]
        self.b_center_default = self.common_behaviour_config["Tracking"]["yCenterDefault"]
        self.b_center_goalie = self.common_behaviour_config["Tracking"]["yCenterGoalie"]

        # just the max and min angles for moving the head
        self.max_tilt = self.common_behaviour_config["Camera"]["maxTilt"]
        self.min_tilt = self.common_behaviour_config["Camera"]["minTilt"]
        self.min_pan = self.common_behaviour_config["Camera"]["minPan"]
        self.max_pan = self.common_behaviour_config["Camera"]["maxPan"]

        self.max_tilt_speed = self.common_behaviour_config["Tracking"]["maxTiltSpeedTracking"]
        self.max_pan_speed = self.common_behaviour_config["Tracking"]["maxPanSpeedTracking"]

        # propably camera angle
        self.angle = self.common_behaviour_config["Camera"]["cameraAngle"]

        # to compute the camera angle in vertical (16:9)
        self.horizontal_factor = self.common_behaviour_config["Camera"]["horizontalFactor"]
        self.vertical_factor = self.common_behaviour_config["Camera"]["verticalFactor"]

    def track_with_values(self, connector, a, b):
        # the goalie wants to track the ball in the upper part of the image, because it will probably come to him
        if connector.get_duty() == "Goalie":
            b_center = self.b_center_goalie
        else:
            b_center = self.b_center_default

        # Get the current pose
        pose = connector.get_pose()

        if not (-self.a_sens < a < self.a_sens):
            goal = pose.head_pan.position + a * self.angle * self.horizontal_factor
            goal = min(self.max_pan, max(self.min_pan, goal))
            pose.head_pan.goal = goal
            pose.head_pan.speed = self.max_pan_speed

        # Ball not centered vertically
        if not (-self.b_sens + b_center < b < self.b_sens + b_center):
            goal = pose.head_tilt.position + b * self.angle * self.vertical_factor
            goal = min(self.max_tilt, max(self.min_tilt, goal))
            pose.head_tilt.goal = goal
            pose.head_tilt.speed = self.max_tilt_speed

        # Set the new pose
        connector.set_pose(pose)

    def perform(self, connector, reevaluate=None):
        raise NotImplementedError


class TrackBall(AbstactTrackObject):
    def perform(self, connector, reevaluate=False):
        # the ball is seen, so we center our view to it
        a = connector.raw_vision_capsule().get_ball_info_legacy_wrapper("a")
        b = connector.raw_vision_capsule().get_ball_info_legacy_wrapper("b")
        self.track_with_values(connector, a, b)


class TrackGoal(AbstactTrackObject):
    def perform(self, connector, reevaluate=None):
        a = connector.raw_vision_capsule().get_goal_infos()[0].x
        b = connector.raw_vision_capsule().get_goal_infos()[0].y
        self.track_with_values(connector, a, b)
