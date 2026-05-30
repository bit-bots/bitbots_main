import math

from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class SecondBallTouchAllowed(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.after_kick_min_ball_movement = self.blackboard.config["after_kick_min_ball_movement"]
        self.other_took_the_first_kick = False
        self.latch = False

    def perform(self, reevaluate=False):
        """
        Determines whether we are confident regarding the ball's position.
        :param reevaluate:
        :return:
        """
        ball_pos = self.blackboard.world_model.get_ball_position_xy()
        kick_off_or_throwin_kick = (
            self.blackboard.misc.kickoff_or_throwin_kick is None or self.blackboard.misc.kickoff_or_throwin_kick
        )
        active_team_players = self.blackboard.team_data.get_number_of_active_field_players(count_goalie=True)
        is_kicking_team = self.blackboard.gamestate.has_kick()
        self.other_took_the_first_kick = (
            self.other_took_the_first_kick or self.blackboard.team_data.is_team_mate_kicking()
        )
        ball_start_position = self.blackboard.misc.ball_movement_detection_start_ball_position

        self.publish_debug_data("other kicked", self.other_took_the_first_kick)
        self.publish_debug_data("kick off or throw in kick", kick_off_or_throwin_kick)
        self.publish_debug_data("active team players", active_team_players)
        self.publish_debug_data("is kicking team", is_kicking_team)
        self.publish_debug_data("ball position", ball_pos)
        self.publish_debug_data("ball start position", ball_start_position)

        if active_team_players < 2:
            return "YES"
        elif self.other_took_the_first_kick:
            self.latch = False
            return "YES"
        elif self.latch:
            return "NO"
        elif not kick_off_or_throwin_kick:
            return "YES"
        elif not is_kicking_team:
            return "YES"
        else:
            if (
                ball_start_position is not None
                and math.hypot(ball_pos[0] - ball_start_position[0], ball_pos[1] - ball_start_position[1])
                > self.after_kick_min_ball_movement
            ):
                self.latch = True
                return "NO"
            else:
                return "YES"

    def get_reevaluate(self):
        return True
