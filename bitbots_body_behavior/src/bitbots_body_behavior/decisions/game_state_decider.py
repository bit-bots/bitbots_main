# -*- coding:utf-8 -*-
import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class GameStateDecider(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GameStateDecider, self).__init__(blackboard, dsd)
        self.game_states = {
            0: 'Initial',
            1: 'Ready',
            2: 'Set',
            3: 'Playing',
            4: 'Finished',
        }

    def perform(self, reevaluate=False):
        if not self.blackboard.gamestate.is_allowed_to_move():
            rospy.logwarn("Not allowed to move")
            return "NOT_ALLOWED_TO_MOVE"

        game_state_number = self.blackboard.gamestate.get_gamestate()
        return self.game_states[game_state_number]

    def get_reevaluate(self):
        """Game state can change during the game"""
        return True
