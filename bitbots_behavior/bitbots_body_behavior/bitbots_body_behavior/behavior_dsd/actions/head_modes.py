from rclpy.task import Future
from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement

from bitbots_msgs.action import LookAt as LookAtROSAction
from bitbots_msgs.msg import HeadMode


class AbstractHeadModeElement(AbstractActionElement):
    """Abstract class used for type hinting"""

    blackboard: BodyBlackboard


class LookAt(AbstractHeadModeElement):
    """Looks at a given point in a given frame."""

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blocking = parameters.get("blocking", True)
        self.finished = not self.blocking

        # Check if action server is running
        server_running = self.blackboard.animation.lookat_action_client.wait_for_server(timeout_sec=0.1)
        if not server_running:
            self.blackboard.node.get_logger().error("LookAt action server not running! Failed to look at ball.")
            self.finished = True
            return

        # Validate parameters
        assert "x" in parameters, "LookAt action missing x parameter"
        assert "y" in parameters, "LookAt action missing y parameter"
        assert "frame" in parameters, "LookAt action missing frame parameter"

        # Create look at goal
        goal = LookAtROSAction.Goal()
        goal.look_at_position.point.x = parameters["x"]
        goal.look_at_position.point.y = parameters["y"]
        goal.look_at_position.point.z = parameters.get("z", 0.0)  # Default to 0.0 if not provided
        goal.look_at_position.header.frame_id = parameters["frame"]

        # Handle action server responses
        def goal_response_callback(future: Future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.blackboard.node.get_logger().warn('Can not perform look at, goal rejected')
                self.finished = True
                return

            def done_callback(future: Future):
                result = future.result().result
                if not result.success:
                    self.blackboard.node.get_logger().warn('Looking at failed')
                self.finished = True

            goal_handle.get_result_async().add_done_callback(done_callback)

        # Call the action server
        self.blackboard.animation.lookat_action_client.send_goal_async(goal).add_done_callback(goal_response_callback)

    def perform(self):
        if self.finished:
            return self.pop()

class TrackBall(AbstractHeadModeElement):
    """Tracks the last known ball position"""

    def perform(self):
        self.blackboard.misc.set_head_duty(HeadMode.TRACK_BALL)
        return self.pop()


class LookAtFieldFeatures(AbstractHeadModeElement):
    """Look generally for all features on the field (ball, goals, corners, center point)"""

    def perform(self):
        self.blackboard.misc.set_head_duty(HeadMode.SEARCH_FIELD_FEATURES)
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
        self.blackboard.misc.set_head_duty(HeadMode.SEARCH_BALL_PENALTY)
        return self.pop()


class LookAtFront(AbstractHeadModeElement):
    """Search in front of the robot"""

    def perform(self):
        self.blackboard.misc.set_head_duty(HeadMode.SEARCH_FRONT)
        return self.pop()
