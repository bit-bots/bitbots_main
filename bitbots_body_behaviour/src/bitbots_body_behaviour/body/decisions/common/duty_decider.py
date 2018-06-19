# -*- coding:utf-8 -*-
"""
DutyDecider
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import rospy
from bitbots_body_behaviour.body.actions.go_away_from_ball import GoAwayFromBall
from bitbots_body_behaviour.body.actions.wait import Wait
from bitbots_body_behaviour.body.decisions.common.go_to_duty_position import GoToDutyPosition
from bitbots_body_behaviour.body.decisions.goalie.goalie_decision import GoaliePositionDecision
from bitbots_body_behaviour.body.decisions.kick_off.kick_off import KickOff
from bitbots_body_behaviour.body.decisions.one_time_kicker.one_time_kicker_decision import OneTimeKickerDecision
from bitbots_body_behaviour.body.decisions.penalty.penalty_kicker_decision import PenaltyKickerDecision
from bitbots_body_behaviour.body.actions.go_to import GoToRelativePosition
from bitbots_connector.capsules.blackboard_capsule import DUTY_GOALIE, DUTY_PENALTYKICKER, DUTY_TEAMPLAYER, DUTY_POSITIONING
from humanoid_league_msgs.msg import Speak, HeadMode, GameState
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule

duty = None  # can be overwriten by the startup script (to force a behaviour)


class DutyDecider(AbstractDecisionModule):
    """
    Decides what kind of behaviour the robot performs
    """

    def __init__(self, connector, _):
        super(DutyDecider, self).__init__(connector)
        self.max_fieldie_time = connector.config["Body"]["Fieldie"]["Defender"]["maxFieldieTime"]
        self.toggle_self_positioning = connector.config["Body"]["Toggles"]["Fieldie"]["trySelfPositioning"]
        self.start_self_pos = None

    def perform(self, connector, reevaluate=False):

        if connector.blackboard.is_frozen() or not connector.gamestate.is_allowed_to_move():
            rospy.logwarn("Not allowed to move")
            return self.push(Wait, 0.1)

        if not connector.blackboard.get_duty():
            if duty is not None:
                # get information about his duty which was set by the startup script
                connector.blackboard.set_duty(duty)
            else:
                connector.blackboard.set_duty(DUTY_TEAMPLAYER)

        if not connector.gamestate.is_game_state_equals(GameState.GAMESTATE_PLAYING):
            # resets all behaviours if the gamestate is not playing, because the robots are positioned again
            if duty is not None:
                connector.blackboard.set_duty(duty)

        ############################
        # # Gamestate related Stuff#
        ############################

        # If we do not Play  or Ready we do nothing
        if connector.gamestate.get_gamestatus() in [GameState.GAMESTATE_INITAL,
                                                    GameState.GAMESTATE_SET,
                                                    GameState.GAMESTATE_FINISHED]:
            rospy.loginfo("Wait for Gamestate")
            # When not playing, the head should look around to find features on the field
            connector.blackboard.set_head_duty(HeadMode.FIELD_FEATURES)
            return self.push(Wait, 0.1)

        # Penalty Shoot but not mine, run away
        elif (connector.config["Body"]["Toggles"]["Fieldie"]["penaltykick_go_away"] and
              connector.blackboard.get_duty() is not DUTY_GOALIE and
              connector.gamestate.has_penalty_kick()):
            return self.push(GoAwayFromBall)

        # Positioning ourself on the Field
        if connector.gamestate.is_game_state_equals(GameState.GAMESTATE_READY):
            # Look for general field features to improve localization
            connector.blackboard.set_head_duty(HeadMode.FIELD_FEATURES)
            if self.toggle_self_positioning:
                return self.push(GoToDutyPosition)
            else:
                # Just walk forward
                return self.push(GoToRelativePosition, (0.5, 0, 0))

        ################################
        # #load cetain part of behaviour
        ################################
        rospy.loginfo("Current duty: " + connector.blackboard.get_duty())

        # If the robot is a OneTimeKicker, kick instead of executing its normal behaviour
        if connector.blackboard.get_is_one_time_kicker():
            return self.push(OneTimeKickerDecision)

        if connector.blackboard.get_duty() == DUTY_TEAMPLAYER:
            return self.push(KickOff)

        elif connector.blackboard.get_duty() == DUTY_GOALIE:
            return self.push(GoaliePositionDecision)

        elif connector.blackboard.get_duty() == DUTY_PENALTYKICKER:
            return self.push(PenaltyKickerDecision)

        ###########################
        # # Other TestStuff
        ###########################

        elif connector.blackboard.get_duty() == DUTY_POSITIONING:
            return self.push(GoToDutyPosition)

        else:
            s = Speak()
            s.text = "Overridden duty not found: %s" % connector.blackboard.get_duty()
            s.priority = Speak.LOW_PRIORITY
            connector.speaker.publish(s)

            raise NotImplementedError

    def get_reevaluate(self):
        return True
