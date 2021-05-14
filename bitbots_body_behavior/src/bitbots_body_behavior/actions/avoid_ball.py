from std_msgs.msg import Bool

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class AvoidBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super(AvoidBall, self).__init__(blackboard, dsd, parameters)

        self.active = parameters.get('active', True)

    def perform(self, reevaluate=False):
        self.publish_debug_data("avoid_ball", self.blackboard.pathfinding.avoid_ball)

        self.blackboard.pathfinding.ball_obstacle_active_pub.publish(Bool(self.active))

        self.blackboard.pathfinding.avoid_ball = self.active

        self.pop()
