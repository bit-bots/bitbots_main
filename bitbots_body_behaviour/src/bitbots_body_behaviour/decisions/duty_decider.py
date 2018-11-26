# -*- coding:utf-8 -*-
"""
DutyDecider
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import rospy
from bitbots_connector.capsules.blackboard_capsule import DUTY_GOALIE, DUTY_PENALTYKICKER, DUTY_TEAMPLAYER, DUTY_POSITIONING
from humanoid_league_msgs.msg import Speak, HeadMode, GameState
from bitbots_dsd.abstract_decision_element import AbstractDecisionElement

duty = None  # can be overwriten by the startup script (to force a behaviour)


class DutyDecider(AbstractDecisionElement):
    """
    Decides what kind of behaviour the robot performs
    """

    def _register(self):
        return ["Nothing", "TeamPlayer", "Goalie", "Test"]

    def __init__(self, blackboard, dsd, parameters=None):
        super(DutyDecider, self).__init__(blackboard, dsd)

        self.max_fieldie_time = blackboard.config["Body"]["Fieldie"]["Defender"]["maxFieldieTime"]
        self.toggle_self_positioning = blackboard.config["Body"]["Toggles"]["Fieldie"]["trySelfPositioning"]
        self.start_self_pos = None

    def perform(self, reevaluate=False):

        if self.blackboard.blackboard.is_frozen() or not self.blackboard.gamestate.is_allowed_to_move():
            rospy.logwarn("Not allowed to move")
            return "Nothing"

        return "TEST"

        if not self.blackboard.blackboard.get_duty():
            if duty is not None:
                # get information about his duty which was set by the startup script
                self.blackboard.blackboard.set_duty(duty)
            else:
                self.blackboard.blackboard.set_duty(DUTY_TEAMPLAYER)

        if not self.blackboard.gamestate.is_game_state_equals(GameState.GAMESTATE_PLAYING):
            # resets all behaviours if the gamestate is not playing, because the robots are positioned again
            if duty is not None:
                self.blackboard.blackboard.set_duty(duty)

        ############################
        # # Gamestate related Stuff#
        ############################

        # If we do not Play  or Ready we do nothing
        if self.blackboard.gamestate.get_gamestatus() in [GameState.GAMESTATE_INITAL,
                                                     GameState.GAMESTATE_SET,
                                                     GameState.GAMESTATE_FINISHED]:
            rospy.loginfo("Wait for Gamestate")
            # When not playing, the head should look around to find features on the field
            self.blackboard.blackboard.set_head_duty(HeadMode.FIELD_FEATURES)
            return "Nothing"

        # Penalty Shoot but not mine, run away
        elif (self.blackboard.config["Body"]["Toggles"]["Fieldie"]["penaltykick_go_away"] and
              self.blackboard.blackboard.get_duty() is not DUTY_GOALIE and
              self.blackboard.gamestate.has_penalty_kick()):
            return self.push(GoAwayFromBall)

        # Positioning ourself on the Field
        if self.blackboard.gamestate.is_game_state_equals(GameState.GAMESTATE_READY):
            # Look for general field features to improve localization
            self.blackboard.blackboard.set_head_duty(HeadMode.FIELD_FEATURES)
            if self.toggle_self_positioning:
                return self.push(GoToDutyPosition)
            else:
                # Just walk forward
                return self.push(GoToRelativePosition, (0.5, 0, 0))

        ################################
        # #load cetain part of behaviour
        ################################
        rospy.loginfo("Current duty: " + self.blackboard.blackboard.get_duty())

        # If the robot is a OneTimeKicker, kick instead of executing its normal behaviour
        if self.blackboard.blackboard.get_is_one_time_kicker():
            return self.push(OneTimeKickerDecision)

        if self.blackboard.blackboard.get_duty() == DUTY_TEAMPLAYER:
            return self.push(KickOff)

        elif self.blackboard.blackboard.get_duty() == DUTY_GOALIE:
            return "Goalie"

        elif self.blackboard.blackboard.get_duty() == DUTY_PENALTYKICKER:
            return self.push(PenaltyKickerDecision)

        ###########################
        # # Other TestStuff
        ###########################

        elif self.blackboard.blackboard.get_duty() == DUTY_POSITIONING:
            return self.push(GoToDutyPosition)

        else:
            s = Speak()
            s.text = "Overridden duty not found: %s" % self.blackboard.blackboard.get_duty()
            s.priority = Speak.LOW_PRIORITY
            self.blackboard.speaker.publish(s)

            raise NotImplementedError

    def get_reevaluate(self):
        return True
