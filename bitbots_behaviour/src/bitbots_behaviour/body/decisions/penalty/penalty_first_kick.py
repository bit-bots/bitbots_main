# -*- coding:utf-8 -*-
"""
PenaltyFirstKick
^^^^^^^^^^^

Works for the first kick for a penalty shoot

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:
* 10.07.14: Erstellt (Marc)
"""
import random
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.behaviour.body.actions.kick_ball import KickBall
from bitbots.util import get_config
from bitbots.util.speaker import say

config = get_config()


class PenaltyFirstKick(AbstractDecisionModule):  # todo not yet refactored 6.12.14.

    def __init__(self, _):
        super(PenaltyFirstKick, self).__init__()
        self.first_run = True
        self.direction = 0
        self.first_left_kick = config["animations"]["flk"]
        self.first_right_kick = config["animations"]["frk"]
        self.first_front_left_kick = config["animations"]["fflk"]
        self.first_front_right_kick = config["animations"]["ffrk"]
        self.toggle_direct_penalty = config["Behaviour"]["Toggles"]["PenaltyFieldie"]["directPenaltyKick"]
        self.penalty_direction = config["Behaviour"]["PenaltyFieldie"]["penaltyDirection"]

    def perform(self, connector, reevaluate=False):

        if self.first_run:
            self.set_direction()

        if self.direction == 0:  # direct shoot
            say("Direct kick on goal")
            if connector.raw_vision_capsule().get_ball_info("v") <= 0:
                self.push(KickBall, init_data="RP")
            else:
                self.push(KickBall, init_data="LP")
        elif self.direction == 1:  # left
            say("first to left")
            connector.animation_capsule().play_animation(self.first_left_kick)
        elif self.direction == 2:  # right
            say("first to right")
            connector.animation_capsule().play_animation(self.first_right_kick)
        elif self.direction == 3:  # short to front
            say("first to front")
            if connector.raw_vision_capsule().get_ball_info("v") <= 0:
                connector.animation_capsule().play_animation(self.first_front_right_kick)
            else:
                connector.animation_capsule().play_animation(self.first_front_left_kick)
        else:
            raise NotImplementedError("This direction is not implented")

        connector.blackboard_capsule().set_first_kick_done()
        connector.blackboard_capsule().set_force_hard_kick()
        connector.set_duty("Striker")
        self.interrupt()

    def set_direction(self):
        self.first_run = False

        if self.penalty_direction != -1:
            # the config tells us the direction
            self.direction = self.penalty_direction
        else:
            # absichltich ist etwas nach vorne schießen standart mäßig aus
            # by purpose is kicking just a litte bit forward normaly turned off
            self.direction = random.randint(0, 2)
