# -*- coding:utf-8 -*-
"""
KickOff
^^^^^^^

Waits if the other team has kickoff till the ball moves or the 10 seconds are over

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:
* 18.07.14: Created (Marc Bestmann)
"""
from bitbots.util.speaker import say
from bitbots.modules.abstract.abstract_decision_module import AbstractDecisionModule
from bitbots.modules.keys import DATA_VALUE_STATE_PLAYING
from bitbots.modules.behaviour.body.actions.wait import Wait


class EnemyKickOff(AbstractDecisionModule):
    def __init__(self, _):
        super(EnemyKickOff, self).__init__()
        self.initilized = False
        self.ball_distance_Saved = 0

    def perform(self, connector, reevaluate=False):
        if not connector.gamestatus_capsule().is_game_state_equals(DATA_VALUE_STATE_PLAYING):
            return self.interrupt()

        if not self.initilized:
            self.initilized = True
            self.ball_distance_saved = connector.raw_vision_capsule().get_ball_info("distance")

        seconds_remaining = connector.gamestatus_capsule().get_secondary_seconds_remaining()
        ball_distance_now = connector.raw_vision_capsule().get_ball_info("distance")

        say("Wait")

        if seconds_remaining == 0 or abs(ball_distance_now - self.ball_distance_saved) > 200:
            say("Enemy moved ball. Can play myself")
            connector.blackboard_capsule().set_enemy_kick_off_done()
            return self.pop()
        else:
            return self.push(Wait, 0.1)
