import rclpy
from bitbots_msgs.action import LookAt
from bitbots_msgs.msg import HeadMode
from dynamic_stack_decider.abstract_action_element import AbstractActionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class AbstractHeadModeElement(AbstractActionElement):
    """Abstract class used for type hinting"""

    blackboard: BodyBlackboard


class LookAtBall(AbstractHeadModeElement):
    """Search for Ball and track it if found"""

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self):
        ball_position = self.blackboard.world_model.get_best_ball_point_stamped()
        server_running = self.blackboard.animation.lookat_action_client.wait_for_server(timeout_sec=1.0)
        if not server_running:
            while not server_running and rclpy.ok():
                self.blackboard.node.get_logger().warn(
                    "Lookat Action Server not running! Lookat cannot work without lookat server!"
                    "Will now wait until server is accessible!",
                    throttle_duration_sec=10.0,
                )
                server_running = self.blackboard.animation.lookat_action_client.wait_for_server(timeout_sec=1)
            if server_running:
                self.blackboard.node.get_logger().warn("Lookat server now running, 'look_at_ball' action will go on.")
            else:
                self.blackboard.node.get_logger().warn("Lookat server did not start. Did not send action.")
                return self.pop()

        goal = LookAt.Goal()
        goal.look_at_position = ball_position
        self.blackboard.animation.lookat_action_client.send_goal_async(goal)
        return self.pop()


class SearchBall(AbstractHeadModeElement):
    """Look for ball"""

    def perform(self):
        self.blackboard.misc.set_head_duty(HeadMode.BALL_MODE)
        return self.pop()


class LookAtFieldFeatures(AbstractHeadModeElement):
    """Look generally for all features on the field (ball, goals, corners, center point)"""

    def perform(self):
        self.blackboard.misc.set_head_duty(HeadMode.FIELD_FEATURES)
        return self.pop()


class LookForward(AbstractHeadModeElement):
    """Simply look directly forward"""

    def perform(self):
        self.blackboard.misc.set_head_duty(HeadMode.LOOK_FORWARD)
        return self.pop()


class DontMoveHead(AbstractHeadModeElement):
    """Don't move the head"""

    def perform(self):
        self.blackboard.misc.set_head_duty(HeadMode.DONT_MOVE)
        return self.pop()


class LookAtBallPenalty(AbstractHeadModeElement):
    """Ball Mode adapted for Penalty Kick"""

    def perform(self):
        self.blackboard.misc.set_head_duty(HeadMode.BALL_MODE_PENALTY)
        return self.pop()


class LookAtFront(AbstractHeadModeElement):
    """Search in front of the robot"""

    def perform(self):
        self.blackboard.misc.set_head_duty(HeadMode.LOOK_FRONT)
        return self.pop()
