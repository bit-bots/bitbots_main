from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from std_msgs.msg import Bool

from bitbots_blackboard.blackboard import BodyBlackboard


class AvoidBall(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard = blackboard
        self.active = parameters.get("active", True)

    def perform(self, reevaluate=False):
        self.blackboard.pathfinding.ball_obstacle_active_pub.publish(Bool(data=self.active))
        self.blackboard.pathfinding.avoid_ball = self.active
        self.publish_debug_data("avoid_ball", self.blackboard.pathfinding.avoid_ball)
        self.pop()


class AvoidBallActive(AvoidBall):
    def __init__(self, blackboard, dsd, parameters):
        parameters["active"] = True
        super().__init__(blackboard, dsd, parameters)


class AvoidBallInactive(AvoidBall):
    def __init__(self, blackboard, dsd, parameters):
        parameters["active"] = False
        super().__init__(blackboard, dsd, parameters)
