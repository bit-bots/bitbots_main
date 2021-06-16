import rospy
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from humanoid_league_msgs.msg import GameState


class AllowedToMove(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(AllowedToMove, self).__init__(blackboard, dsd, parameters)
        self.ball_lost_time = rospy.Duration.from_sec(self.blackboard.config['ball_lost_time'])
        self.kickoff_min_ball_movement = rospy.get_param("behavior/body/kickoff_min_ball_movement")

    def perform(self, reevaluate=False):
        """
        Determines whether the robot is penalized or was just unpenalized.
        :param reevaluate:
        :return:
        """
        self.publish_debug_data("Seconds since unpenalized",
                                self.blackboard.gamestate.get_seconds_since_unpenalized())
        if self.blackboard.gamestate.get_is_penalized() \
                or self.blackboard.gamestate.get_gamestate() == GameState.GAMESTATE_FINISHED:
            return 'NO_MOVEMENT'
        elif self.blackboard.gamestate.get_gamestate() in (GameState.GAMESTATE_INITAL, GameState.GAMESTATE_SET):
            return 'ONLY_HEAD'
        elif self.blackboard.gamestate.get_secondary_state() != GameState.STATE_NORMAL \
                and self.blackboard.gamestate.get_secondary_state_mode() in (GameState.MODE_PREPARATION,
                                                                             GameState.MODE_END):
            return 'ONLY_HEAD'
        elif self.blackboard.gamestate.get_gamestate() == GameState.GAMESTATE_READY:
            return 'NORMAL'
        elif self.blackboard.gamestate.get_gamestate() == GameState.GAMESTATE_PLAYING:
            if not self.blackboard.gamestate.has_kickoff():
                if self.blackboard.gamestate.get_secondary_seconds_remaining() == 0:
                    # time is up for the other team
                    return 'NORMAL'
                ball_pos = self.blackboard.world_model.get_ball_position_xy()
                # check if this is a normal kickoff
                if self.blackboard.gamestate.free_kick_kickoff_team is None:
                    # if we know where the ball is and that it moved, we can play too
                    if rospy.Time.now() - self.blackboard.world_model.ball_last_seen() < self.ball_lost_time and (
                            abs(ball_pos[0]) > self.kickoff_min_ball_movement or
                            abs(ball_pos[1]) > self.kickoff_min_ball_movement):
                        return 'NORMAL'
                # we need to wait for now
                return 'ONLY_HEAD'
            else:
                if self.blackboard.gamestate.get_seconds_since_unpenalized() < 1:
                    return 'JUST_UNPENALIZED'
                else:
                    return 'NORMAL'
        else:
            rospy.logerr("UNKNOWN STATE !!!")
            return 'NORMAL'

    def get_reevaluate(self):
        return True
