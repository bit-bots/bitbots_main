# -*- coding:utf-8 -*-
"""
EnemyKickOff
^^^^^^^^^^^^

Waits if the other team has kickoff till the ball moves or the 10 seconds are over

.. moduleauthor:: Marc Bestmann <0bestman@informatik.uni-hamburg.de>

History:
* 18.07.14: Created (Marc Bestmann)
"""
from bitbots_body_behaviour.actions.wait import Wait
from humanoid_league_msgs.msg import GameState
from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement


class EnemyKickOff(AbstractDecisionElement):
    def __init__(self, connector):
        super(EnemyKickOff, self).__init__(connector)
        self.initialized = False
        self.ball_distance_saved = 0

    def perform(self, connector, reevaluate=False):
        if not connector.gamestatus.is_game_state_equals(GameState.GAMESTATE_PLAYING):
            return self.interrupt()

        if not self.initialized:
            self.initialized = True
            self.ball_distance_saved = connector.world_model.get_ball_distance()

        seconds_remaining = connector.gamestatus.get_secondary_seconds_remaining()
        ball_distance_now = connector.world_model.get_ball_distance()

        if seconds_remaining == 0 or abs(ball_distance_now - self.ball_distance_saved) > 0.2:
            connector.blackboard.set_enemy_kick_off_done()
            return self.pop()
        else:
            return self.push(Wait, 0.1)
