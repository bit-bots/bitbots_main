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
        # todo this is a temporary hack to make GUI work
        if state_number == 0:
            return "NORMAL"
        elif state_number == 1:
            return "PENALTYSHOOT"
        elif state_number == 2:
            return "OVERTIME"
        elif state_number == 3:
            return "TIMEOUT"
        elif state_number == 4:
            return "DIRECT_FREEKICK"
        elif state_number == 5:
            return "INDIRECT_FREEKICK"
        elif state_number == 6:
            return "PENALTYKICK"
        elif state_number == 7:
            return "CORNER_KICK"
        elif state_number == 8:
            return "GOAL_KICK"
        elif state_number == 9:
            return "THROW_IN"

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
        if self.team_id == self.blackboard.gamestate.get_secondary_team():
            return 'OUR'
        return 'OTHER'

    def get_reevaluate(self):
        """
        Secondary state Team can change during the game
        """
        return True
