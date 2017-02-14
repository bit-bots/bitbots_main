"""
ConfirmGoal
^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

"""

from bitbots_head_behaviour.head_connector import HeadConnector
from bitbots_stackmachine.abstract_init_action_module import AbstractInitActionModule


class AbstactTrackObject(AbstractInitActionModule):
    """
    Confirmed either the Ball, OwnGoal or EnemyGoal by passing it to the init arg
    """

    def __init__(self, connector: HeadConnector, args=None):
        super(AbstactTrackObject, self).__init__(connector, args)

        # this influences how precise the ball has to be in the center to make the head move
        self.a_sens = connector.config["Tracking"]["xSensivity"]
        self.b_sens = connector.config["Tracking"]["ySensivity"]
        self.b_center_default = connector.config["Tracking"]["yCenterDefault"]
        self.b_center_goalie = connector.config["Tracking"]["yCenterGoalie"]

        # just the max and min angles for moving the head
        self.max_tilt = connector.config["Camera"]["maxTilt"]
        self.min_tilt = connector.config["Camera"]["minTilt"]
        self.min_pan = connector.config["Camera"]["minPan"]
        self.max_pan = connector.config["Camera"]["maxPan"]

        self.max_tilt_speed = connector.config["Tracking"]["maxTiltSpeedTracking"]
        self.max_pan_speed = connector.config["Tracking"]["maxPanSpeedTracking"]

        # propably camera angle
        self.angle = connector.config["Camera"]["cameraAngle"]

        # to compute the camera angle in vertical (16:9)
        self.horizontal_factor = connector.config["Camera"]["horizontalFactor"]
        self.vertical_factor = connector.config["Camera"]["verticalFactor"]

    def track_with_values(self, connector, a, b):
        # the goalie wants to track the ball in the upper part of the image, because it will probably come to him
        if connector.get_duty() == "Goalie":
            b_center = self.b_center_goalie
        else:
            b_center = self.b_center_default

        # Get the current positions
        curren_pan_pos, current_tilt_pos = connector.head.get_current_head_pos()

        if not (-self.a_sens < a < self.a_sens):
            goal = curren_pan_pos + a * self.angle * self.horizontal_factor
            goal = min(self.max_pan, max(self.min_pan, goal))
            head_pan_goal = goal
        else:
            head_pan_goal = curren_pan_pos

        # Ball not centered vertically
        if not (-self.b_sens + b_center < b < self.b_sens + b_center):
            goal = current_tilt_pos + b * self.angle * self.vertical_factor
            goal = min(self.max_tilt, max(self.min_tilt, goal))
            head_tilt_goal = goal
        else:
            head_tilt_goal = current_tilt_pos

        connector.head.send_motor_goals(head_pan_goal, self.max_pan_speed, head_tilt_goal, self.max_tilt_speed)

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
