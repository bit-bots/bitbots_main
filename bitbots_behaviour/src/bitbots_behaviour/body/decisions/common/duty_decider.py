# -*- coding:utf-8 -*-
"""
DutyDecider
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

History:
* 29.11.13: Created (Martin Poppinga)
* 27.8.14: Updated (Lars Thoms)
* 04.12.14: Refactored (Nils Rokita & Marc Bestmann)
"""

from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.focus_goal import FocusEnemyGoal
from bitbots.modules.behaviour.body.actions.go_to_absolute_position import GoToAbsolutePosition
from bitbots.modules.behaviour.body.actions.plain_walk_action import PlainWalkAction
from bitbots.modules.behaviour.body.actions.testing.test_walking_dynamic import TestWalkingDynamic
from bitbots.modules.behaviour.body.actions.testing.test_walking_static import TestWalkingStatic
from bitbots.modules.behaviour.body.actions.wait import Wait
from bitbots.modules.behaviour.body.decisions.calibratevision.calibrate_vision import CalibrateVision
from bitbots.modules.behaviour.body.decisions.common.go_to_duty_position import GoToDutyPosition
from bitbots.modules.behaviour.body.decisions.common.role_decider import RoleDecider
from bitbots.modules.behaviour.body.decisions.goalie.goalie_decision import GoalieDecision
from bitbots.modules.behaviour.body.decisions.kick_off.kick_off import KickOff
from bitbots.modules.behaviour.body.decisions.one_time_kicker.one_time_kicker_decision import OneTimeKickerDecision
from bitbots.modules.behaviour.body.decisions.penalty.penalty_kicker_decision import PenaltyKickerDecision
from bitbots.modules.behaviour.body.decisions.technical_challenges.throw_in_entry_point import ThrowInEntryPoint
from bitbots.modules.behaviour.modell.capsules.walking_capsule import WalkingCapsule
from bitbots.modules.keys import DATA_VALUE_STATE_PLAYING, DATA_VALUE_STATE_READY, DATA_VALUE_STATE_SET, \
    DATA_VALUE_STATE_FINISHED, DATA_VALUE_STATE_INITIAL
from bitbots.util.config import get_config
from bitbots.util.speaker import say

duty = None  # can be overwriten by the startup script (to force a behaviour)


class DutyDecider(AbstractDecisionModule):
    """
    Decides what kind of beahvoiur the robot performs
    """

    def __init__(self, _):
        super(DutyDecider, self).__init__()
        config = get_config()["Behaviour"]
        self.max_fieldie_time = config["Fieldie"]["Defender"]["maxFieldieTime"]
        self.toggle_self_positioning = config["Toggles"]["Fieldie"]["trySelfPositioning"]

    def perform(self, connector, reevaluate=False):

        if connector.blackboard_capsule().is_frozen():
            return

        if not connector.get_duty():
            if duty is not None:
                # get information about his duty which was set by the startup script
                connector.set_duty(duty)
            else:
                connector.set_duty("TeamPlayer")

        if not connector.gamestatus_capsule().is_game_state_equals(DATA_VALUE_STATE_PLAYING):
            # resets all behaviours if the gamestate is not playing, because the robots are positioned again
            if duty is not None:
                connector.set_duty(duty)

        ############################
        # # Gamestate related Stuff#
        ############################

        # If we do not Play  or Ready we do nothing
        if connector.gamestatus_capsule().get_gamestatus() in [DATA_VALUE_STATE_INITIAL,
                                                               DATA_VALUE_STATE_SET,
                                                               DATA_VALUE_STATE_FINISHED]:
            return self.push(Wait, 0.1)

        # Positioning ourself on the Field
        if self.toggle_self_positioning:
            if connector.gamestatus_capsule().is_game_state_equals(DATA_VALUE_STATE_READY):  # Todo checking if working
                return self.push(GoToDutyPosition)

        ################################
        # #load cetain part of behaviour
        ################################

        if connector.get_duty() in ["TeamPlayer"]:
            return self.push(KickOff)

        elif connector.get_duty() == "Goalie":
            return self.push(GoalieDecision)

        elif connector.get_duty() == "OneTimeKicker":
            return self.push(OneTimeKickerDecision)

        # this should be normally not used just for debug or emergency
        elif connector.get_duty() in ["Defender", "Striker", "Center", "Supporter"]:
            return self.push(RoleDecider, connector.get_duty())

        elif connector.get_duty() == "PenaltyKickFieldie":
            return self.push(PenaltyKickerDecision)

        ###########################
        # # Tecnical Chelanges
        ###########################

        elif connector.get_duty() == "ThrowIn":
            return self.push(ThrowInEntryPoint)

        ###########################
        # # Other TestStuff
        ###########################

        elif connector.get_duty() == "TestWalkingStatic":
            return self.push(TestWalkingStatic)

        elif connector.get_duty() == "TestWalkingDynamic":
            return self.push(TestWalkingDynamic)

        elif connector.get_duty() == "GoToPosition":
            return self.push(GoToDutyPosition)

        elif connector.get_duty() == "Positionate":
            return self.push(GoToAbsolutePosition, [50, 50, 30])

        elif connector.get_duty() == "Nothing":
            return self.push(Wait)

        elif connector.get_duty() == "Stay":
            connector.walking_capsule().start_walking_plain(0, 0, 0)
            return

        elif connector.get_duty() == "FocusGoal":
            return self.push(FocusEnemyGoal)

        elif connector.get_duty() == "CalibrateVision":
            return self.push(CalibrateVision)

        else:
            say("Overridden duty not found: %s" % connector.get_duty())
            raise NotImplementedError

    def get_reevaluate(self):
        return True
