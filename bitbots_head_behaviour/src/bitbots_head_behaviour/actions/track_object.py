"""
ConfirmGoal
^^^^^^^^^^

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

"""
import rospy

from bitbots_common.connector.connector import HeadConnector
from bitbots_stackmachine.abstract_init_action_module import AbstractInitActionModule


class AbstactTrackObject(AbstractInitActionModule):
    """
    Confirmed either the Ball, OwnGoal or EnemyGoal by passing it to the init arg
    """

    def __init__(self, connector: HeadConnector, args=None):
        super(AbstactTrackObject, self).__init__(connector, args)

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

    def track_with_values(self, connector: HeadConnector, x, y):
        a = ((x / connector.head.cam_info[0]) - 0.5) * 2
        b = ((y / connector.head.cam_info[1]) - 0.5) * -2
        rospy.logdebug("rela: %f relb: %f " % (a, b))


        # the goalie wants to track the ball in the upper part of the image, because it will probably come to him
        if connector.head.get_headmode() == "Goalie":  # todo: mit headmodes richtig lösen
            b_center = self.b_center_goalie
        else:
            b_center = self.b_center_default

        # Get the current positions
        curren_pan_pos, current_tilt_pos = connector.head.get_current_head_pos()
        rospy.logdebug("OldTiltgoal: %f" % current_tilt_pos)
        rospy.logdebug("OldPangoal: %f" % curren_pan_pos)
        if not (-self.a_sens < a < self.a_sens):
            goal = curren_pan_pos + a * (self.angle/2.0) * self.horizontal_factor
            goal = min(self.max_pan, max(self.min_pan, goal))
            head_pan_goal = goal
        else:
            head_pan_goal = curren_pan_pos

        # Ball not centered vertically
        if not (-self.b_sens + b_center < b < self.b_sens + b_center):
            goal = current_tilt_pos + b * (self.angle/2.0) * self.vertical_factor
            goal = min(self.max_tilt, max(self.min_tilt, goal))
            head_tilt_goal = goal

        else:
            head_tilt_goal = current_tilt_pos
        rospy.logdebug("Tiltgoal: %f" % head_tilt_goal)
        rospy.logdebug("Pangoal: %f" % head_pan_goal)

        connector.head.send_motor_goals(head_pan_goal, self.max_pan_speed, head_tilt_goal, self.max_tilt_speed)

    def perform(self, connector, reevaluate=None):
        raise NotImplementedError


class TrackBall(AbstactTrackObject):
    def perform(self, connector: HeadConnector, reevaluate=False):
        # the ball is seen, so we center our view to it
        x, y = connector.head.bestball_in_image
        if x is not None:
            self.track_with_values(connector, x, y)
        else:
            rospy.loginfo("No ball found to track")


class TrackGoal(AbstactTrackObject):
    def perform(self, connector: HeadConnector, reevaluate=None):
        raise NotImplementedError
        a = connector.vision.get_goal_infos()[0].x
        b = connector.vision.get_goal_infos()[0].y
        self.track_with_values(connector, a, b)
