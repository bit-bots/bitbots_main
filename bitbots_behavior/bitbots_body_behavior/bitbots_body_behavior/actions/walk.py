from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from geometry_msgs.msg import Twist
from rclpy.duration import Duration


class WalkForward(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd)
        self.time = parameters.get("time", 0.5)
        self.start_time = self.blackboard.node.get_clock().now()

    def perform(self, reevaluate=False):
        if self.blackboard.node.get_clock().now() - self.start_time < Duration(seconds=self.time):
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.2
            self.blackboard.pathfinding.direct_cmd_vel_pub.publish(cmd_vel)
        else:
            self.pop()
