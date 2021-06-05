import rospy
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from humanoid_league_msgs.msg import GameState


class AllowedToMove(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(AllowedToMove, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Determines whether the robot is penalized or was just unpenalized.
        :param reevaluate:
        :return:
        """
        if self.blackboard.gamestate.get_is_penalized() or self.blackboard.gamestate.get_gamestate() == GameState.GAMESTATE_FINISHED:
            return 'NO_MOVEMENT'
        elif self.blackboard.gamestate.get_gamestate() in (GameState.GAMESTATE_INITAL, GameState.GAMESTATE_SET):
            return 'ONLY_HEAD'
        elif self.blackboard.gamestate.get_secondary_state() in (GameState.STATE_DIRECT_FREEKICK,
                                                       GameState.STATE_INDIRECT_FREEKICK,
                                                       GameState.STATE_PENALTYKICK,
                                                       GameState.STATE_CORNER_KICK,
                                                       GameState.STATE_GOAL_KICK,
                                                       GameState.STATE_THROW_IN) and \
                self.blackboard.get_secondary_state_mode() in (GameState.MODE_PREPARATION, GameState.MODE_END):
            return 'ONLY_HEAD'
        elif self.blackboard.gamestate.get_gamestate() == GameState.GAMESTATE_PLAYING:
            if not self.blackboard.gamestate.has_kickoff() and self.blackboard.secondary_seconds_remaining() != 0:
                return 'ONLY_HEAD'
            else:
                self.publish_debug_data("Seconds since unpenalized",
                                        self.blackboard.gamestate.get_seconds_since_unpenalized())
                if self.blackboard.gamestate.get_seconds_since_unpenalized() < 1:
                    return 'JUST_UNPENALIZED'
                else:
                    return 'NORMAL'
        else:
            rospy.logerr("UNKNOWN STATE !!!")
            return 'NORMAL'

    def get_reevaluate(self):
        return True
