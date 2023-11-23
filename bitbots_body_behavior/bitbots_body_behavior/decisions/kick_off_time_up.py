from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class KickOffTimeUp(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.kickoff_min_ball_movement = self.blackboard.config["kickoff_min_ball_movement"]

    def perform(self, reevaluate=False):
        if self.blackboard.gamestate.has_kickoff():
            self.publish_debug_data("Reason", "Our kick off")
            return "YES"
        else:
            if self.blackboard.gamestate.get_secondary_seconds_remaining() == 0:
                self.publish_debug_data("Reason", "Opp kick off time ended")
                # time is up for the other team
                return "YES"
            ball_pos = self.blackboard.world_model.get_ball_position_xy()
            # check if this is a normal kickoff
            if self.blackboard.gamestate.free_kick_kickoff_team is None:
                # if we know where the ball is and that it moved, we can play too
                if (
                    abs(ball_pos[0]) > self.kickoff_min_ball_movement
                    or abs(ball_pos[1]) > self.kickoff_min_ball_movement
                ):
                    self.publish_debug_data("Reason", "Opp kick off ball moved")
                    return "YES"
            else:
                # this is some kind of free kick and we may want to act differently
                self.publish_debug_data("Reason", "Opp has free kick")
                return "NO_FREEKICK"
            self.publish_debug_data("Reason", "Opp has kick off")
            # we need to wait for now
            return "NO_NORMAL"

    def get_reevaluate(self):
        return True
