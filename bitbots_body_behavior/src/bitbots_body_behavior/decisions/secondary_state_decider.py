# -*- coding:utf-8 -*-
import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class SecondaryStateDecider(AbstractDecisionElement):
    """
    Decides in which secondary state the game is currently in. The mode of the secondary state is handled in the
    game controller receiver, so the behavior does ont need to deal with this.
    """
    def __init__(self, blackboard, dsd, parameters=None):
        super(SecondaryStateDecider, self).__init__(blackboard, dsd, parameters)
        self.secondary_game_states = {
            0: 'NORMAL',
            1: 'PENALTYSHOOT',  # should not happen during halftime or extra time
            2: 'OVERTIME',
            3: 'TIMEOUT',
            4: 'DIRECT_FREEKICK',
            5: 'INDIRECT_FREEKICK',
            6: 'PENALTYKICK',
            7: 'CORNER_KICK',
            8: 'GOAL_KICK',
            9: 'THROW_IN',
        }

    def perform(self, reevaluate=False):
        state_number = self.blackboard.gamestate.get_secondary_state()
        return self.secondary_game_states[state_number]

    def get_reevaluate(self):
        """
        Secondary game state can change during the game
        """
        return True


class SecondaryStateTeamDecider(AbstractDecisionElement):
    """
    Decides if our team or the other team is allowed to execute the secondary state.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(SecondaryStateTeamDecider, self).__init__(blackboard, dsd)
        self.team_id = self.blackboard.gamestate.team_id

    def perform(self, reevaluate=False):
        if self.blackboard.gamestate.has_kickoff():
            return 'OUR'
        return 'OTHER'

    def get_reevaluate(self):
        """
        Secondary state Team can change during the game
        """
        return True
