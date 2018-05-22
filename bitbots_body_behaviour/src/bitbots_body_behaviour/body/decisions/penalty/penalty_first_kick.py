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

from bitbots_body_behaviour.body.actions.kick_ball import KickBall
from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from humanoid_league_msgs.msg import TeamData


class PenaltyFirstKick(AbstractDecisionModule):

    def __init__(self, connector):
        super(PenaltyFirstKick, self).__init__(connector)
        self.first_run = True
        self.direction = 0
        self.first_left_kick = connector.animation.config["Penalty"]["leftKick"]
        self.first_right_kick = connector.animation.config["Penalty"]["rightKick"]
        self.first_front_left_kick = connector.animation.config["Penalty"]["frontLeftKick"]
        self.first_front_right_kick = connector.animation.config["Penalty"]["frontRightKick"]
        self.toggle_direct_penalty = connector.config["Body"]["Toggles"]["PenaltyFieldie"]["directPenaltyKick"]
        self.penalty_direction = connector.config["Body"]["PenaltyFieldie"]["penaltyDirection"]

    def perform(self, connector, reevaluate=False):

        if self.first_run:
            self.set_direction()

        if self.direction == 0:  # direct shoot
            if connector.world_model.get_ball_position_uv()[1] <= 0:
                self.push(KickBall, init_data="RIGHT_KICK_STRONG")
            else:
                self.push(KickBall, init_data="LEFT_KICK_STRONG")
        elif self.direction == 1:  # left
            connector.animation.play_animation(self.first_left_kick)
        elif self.direction == 2:  # right
            connector.animation.play_animation(self.first_right_kick)
        elif self.direction == 3:  # short to front
            if connector.world_model.get_ball_position_uv()[1] <= 0:
                connector.animation.play_animation(self.first_front_right_kick)
            else:
                connector.animation.play_animation(self.first_front_left_kick)
        else:
            raise NotImplementedError("This direction is not implemented")

        connector.blackboard.set_first_kick_done()
        connector.blackboard.set_force_hard_kick()
        connector.team_data.set_role(TeamData.ROLE_STRIKER)

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
