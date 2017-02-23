"""
GoalieBehaviour_Dynamic
^^^^^^^^^^^
This Class represents the actual Behaviour for the Goalie.
It's called dynamic because the Goalie will reposition himself when the ball is too far left or right of him,
so he shortens the angle to the ball and covers as much from the goal as possible.

@author: Daniel Speck
Created on 20.08.2014
"""
import math
import time
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.plain_walk_action import PlainWalkAction
from bitbots.modules.behaviour.modell.capsules.walking_capsule import WalkingCapsule
from bitbots.util import get_config
from bitbots.debug.test.debug_log import DebugLog

from bitbots_common.connector.connector import BodyConnector


class GoalieBehaviourDynamic(AbstractDecisionModule):
    # threshold angle (if ball is seen under an angle greater than the threshold
    # the bitbot should correct his position)
    ACTIONANGLE = 20
    WALKING_THRESHOLD = 1000

    # For debugging
    G_DOLOG = True  # should logging be activated?
    G_LOGFILE = "GoalieDynamic.log"  # name of the logfile
    G_logcount = 0  # counting the logs
    G_lastcall = int(time.time())  # when was the last log?
    G_DEBUGTIMETHRESOLD = 1  # how often should be logged? (in seconds)

    # Variables
    goalCenter = (9000, 0)
    goalCenter_old = (9000, 0)

    def __init__(self, connector: BodyConnector, _):
        super(GoalieBehaviourDynamic, self).__init__(connector)
        self.log = DebugLog(self.G_LOGFILE)

    def own_debug(self, connector):
        # For debugging purposes
        if self.G_DOLOG:

            self.G_logcount += 1

            # Trennstrich und Zaehlerangabe
            self.log.addLineseperator()
            self.log.newLine()
            self.log.add("logcount: ")
            self.log.add(self.G_logcount)
            self.log.newLine()
            self.log.addLineseperator()
            self.log.newLine()

            # Ballinformationen (raw)
            self.log.add("Ballinformation (raw):")
            self.log.newLine()
            self.log.add("u=")
            self.log.addSpacesLeft(str(round(connector.use_raw_vision_info_capsule().get_ball_info("u"), 2)), 9)
            self.log.addSeperator()
            self.log.add("v=")
            self.log.addSpacesLeft(str(round(connector.use_raw_vision_info_capsule().get_ball_info("v"), 2)), 9)
            self.log.newLine()
            self.log.add("distance=")
            self.log.addSpacesLeft(str(round(connector.use_raw_vision_info_capsule().get_ball_info("distance"), 2)), 9)
            self.log.addSeperator()
            self.log.add("BallAngle= ")
            self.log.addSpacesLeft(str(round(connector.use_raw_vision_info_capsule().get_ball_angle(), 2)), 7)
            self.log.newLine()
            self.log.add("rating=")
            self.log.addSpacesLeft(str(round(connector.use_raw_vision_info_capsule().get_ball_info("rating"), 4)), 10)
            self.log.newLine()

            # Ballinformationen (simpleFilter)
            self.log.addSmallLineseperator()
            self.log.newLine()
            self.log.add("Ballinformation (simpleFilter):")
            self.log.newLine()
            self.log.add("u=")
            self.log.addSpacesLeft(
                str(round(connector.use_filtered_vision_info_capsule().get_simple_filtered_ball_info().u, 2)), 9)
            self.log.addSeperator()
            self.log.add("v=")
            self.log.addSpacesLeft(
                str(round(connector.use_filtered_vision_info_capsule().get_simple_filtered_ball_info().v, 2)), 9)
            self.log.newLine()
            self.log.add("distance=")
            self.log.addSpacesLeft(
                str(round(connector.use_filtered_vision_info_capsule().get_simple_filtered_ball_info().distance, 2)), 9)
            self.log.addSeperator()
            self.log.add("angle=")
            self.log.addSpacesLeft(
                str(round(connector.use_filtered_vision_info_capsule().get_simple_filtered_ball_info().angle, 2)), 7)
            self.log.newLine()
            self.log.add("uestimate=")
            self.log.addSpacesLeft(
                str(round(connector.use_filtered_vision_info_capsule().get_simple_filtered_ball_info().uestimate, 2)),
                9)
            self.log.addSeperator()
            self.log.add("vestimate=")
            self.log.addSpacesLeft(
                str(round(connector.use_filtered_vision_info_capsule().get_simple_filtered_ball_info().vestimate, 2)),
                9)
            self.log.newLine()

            # Torkoordinaten
            if not (connector.use_filtered_vision_info_capsule().get_center_of_seen_goal()
                    is None):
                log_g_c = connector.use_filtered_vision_info_capsule().get_center_of_seen_goal()
            else:
                log_g_c = ("Not Found", "Not Found")
            self.log.add("Tormittelpunkt: ")
            self.log.add("u=")
            self.log.add(str(log_g_c[0]))
            self.log.addSeperator()
            self.log.add("v=")
            self.log.add(str(log_g_c[1]))
            self.log.newLine()
            self.log.logWrite()

    def perform(self, connector, reevaluate=False):

        # config laden
        config = get_config()

        # is last debug-call older than 1 second? (respectively older than G_DEBUGTIMETHRESHOLD)
        if config["Behaviour"]["Goalie"]["goalieDynamicDoLog"]:
            if int(int(time.time()) - self.G_lastcall) > self.G_DEBUGTIMETHRESOLD:
                self.G_lastcall = int(time.time())  # update last call time
                self.own_debug(connector)

        # --- Vars ---
        # rating
        rating = connector.use_raw_vision_info_capsule().get_ball_info("rating")
        # rating = connector.use_filtered_vision_info_capsule().get_simple_filtered_ball_info().rating
        # ball angle
        # ball_angle = connector.use_raw_vision_info_capsule().get_ball_angle()
        ball_angle = connector.use_filtered_vision_info_capsule().get_simple_filtered_ball_info().angle
        # get the estimated values from the BallDataInfoFilterModule
        uestimate, vestimate = connector.use_filtered_vision_info_capsule().get_uv_estimate()

        # Do nothing if rating is bad
        if rating > 30:
            return

        # Goal center (if seen)
        # Tuple: u,v
        # ToDo: File debuggen "FilteredVisionInfoCapsule.py"
        # , line 66, in get_complete_goal_seen
        if not (connector.use_filtered_vision_info_capsule().get_center_of_seen_goal() is None):
            self.goalCenter_old = tuple(self.goalCenter)
            self.goalCenter = connector.use_filtered_vision_info_capsule().get_center_of_seen_goal()
        else:
            self.goalCenter = tuple(self.goalCenter_old)

        # Distance to the goal center
        dist_goal_center = math.fabs(self.goalCenter[1])

        # The bitbot shouldnt move to the post but stop at some threshold
        # (config["field"]["goal-width"] / 2.0) -> Half goal size
        # (config["field"]["goal-width"] / 20) -> 5% of the goal size
        self.WALKING_THRESHOLD = (config["field"]["goal-width"] / 2.0) - (config["field"]["goal-width"] / 20)

        # If ball is estimated to pass goal-line
        # Check the angle to the ball and correct position, if greater than threshold
        if math.fabs(ball_angle) < self.ACTIONANGLE:
            return
        # Check distance to the post and move or don't (if post is close or not)
        elif self.WALKING_THRESHOLD > dist_goal_center:
            if ball_angle < 0:
                return self.push(PlainWalkAction,
                                 [[WalkingCapsule.ZERO, WalkingCapsule.ZERO, WalkingCapsule.SLOW_SIDEWARDS_LEFT, 3]])
            else:
                return self.push(PlainWalkAction,
                                 [[WalkingCapsule.ZERO, WalkingCapsule.ZERO, WalkingCapsule.SLOW_SIDEWARDS_RIGHT, 3]])
        else:
            return
