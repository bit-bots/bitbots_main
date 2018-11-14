"""
CloseBall
^^^^^^^^^

.. moduleauthor:: Martin Poppinga <robocup@poppinga.xyz>
"""
from humanoid_league_msgs.msg import HeadMode

from bitbots_dsd.abstract_decision_element import AbstractDecisionElement


class CloseBall(AbstractDecisionElement):
    """
    Test if the ball is in kick distance
    """

    def __init__(self, blackboard, _):
        self.last_goalie_dist = 0
        self.last_goalie_dist_time = 0
        self.max_kick_distance = blackboard.config["Body"]["Fieldie"]["kickDistance"]
        self.min_kick_distance = blackboard.config["Body"]["Fieldie"]["minKickDistance"]
        self.config_kickalign_v = blackboard.config["Body"]["Fieldie"]["kickAlign"]

    def perform(self, connector, reevaluate=False):
        # When the ball is seen, the robot should switch between looking to the ball and the goal
        connector.blackboard.set_head_duty(HeadMode.BALL_GOAL_TRACKING)  # todo in action
        # if the robot is near to the ball
        if self.min_kick_distance < connector.world_model.get_ball_position_uv()[0] <= self.max_kick_distance \
                and connector.world_model.get_ball_distance() <= self.max_kick_distance * 5.0:
            # TODO config
            self.action(connector)
        else:
            self.go(connector)

    def action(self, connector):
        return "CLOSE"

    def go(self, connector):
        return "FAR"

    def get_reevaluate(self):
        return True

    def _register(self):
        return ["CLOSE", "FAR"]


class CloseBallPenaltyKick:
    # TODO in extra verhalten, das hat nichts mit der Entscheidung zu tun ob der Ball gerade nah ist
    def __init__(self, connector):
        super(CloseBallPenaltyKick, self).__init__(connector)
        self.toggle_direct_penalty = connector.config["Body"]["Toggles"]["PenaltyFieldie"]["directPenaltyKick"]
        self.use_special_pathfinding = connector.config["Body"]["Toggles"]["PenaltyFieldie"]["useSpecialPathfinding"]

    def action(self, connector):
        if not self.toggle_direct_penalty and not connector.blackboard.get_first_kick_done():
            # todo first kick müsste eigenlich nicht extra abgecheckt werden, später raus nehmen
            # the penalty kick is different for now
            return self.push(PenaltyFirstKick)
        else:
            if abs(connector.world_model.get_ball_position_uv()[1]) > self.config_kickalign_v:
                return self.push(GoToBall)
            else:
                return self.push(KickDecisionPenaltyKick)

    def go(self, connector):
        return self.push(GoToBall)


class CloseBallGoalie:
    #Todo auch extra
    def perform(self, connector, reevaluate=False):
        if connector.blackboard_capsule().has_goalie_kicked():
            return self.interrupt()
        super(AbstractCloseBall, self).perform(connector, reevaluate)  # perform an normal CloseBall
