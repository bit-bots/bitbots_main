import math
from typing import Optional

import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time

from bitbots_msgs.action import Kick
from bitbots_rl_motion.handlers import Handler
from bitbots_rl_motion.handlers.tf_utils import wrap_to_pi, yaw_in_frame


class AmpKickHandler(Handler):
    """Runs the kick action and builds the task command of the AMP kick policy.

    The policy walks to the ball and kicks it on its own, so it is not told a
    velocity. It is told where it should kick the ball to and how hard:

      * ``kick_dir``: the target kick direction as a unit vector in the yaw frame of
        the robot. The goal gives that direction in the body frame at the time the
        goal is received. It is anchored in the odom frame right away and expressed
        back into the current body frame in every step, so it keeps pointing at the
        same direction in the world while the robot turns towards the ball.
      * ``kick_strong``: whether a strong or a weak kick was asked for. During
        training this is a binary command, so the requested strength is compared
        against a threshold instead of being passed through.

    The policy is only run while a kick goal is live. The goal is held for its
    timeout and then finishes, as the policy has no notion of a completed kick.
    """

    def __init__(self, node):
        self._node = node

        self._default_timeout = float(node.get_parameter("command.kick_timeout").value)
        self._strong_kick_min_strength = float(node.get_parameter("command.strong_kick_min_strength").value)

        self._odom_frame = str(node.get_parameter("command.odom_frame").value)
        self._base_frame = str(node.get_parameter("command.base_frame").value)

        self._active = False
        self._abort_requested = False
        self._kick_strong = 0.0
        # Requested direction in the body frame at the time the goal was received,
        # used as the fallback if we could not anchor it
        self._kick_dir_body = 0.0
        # The anchored direction in the odom frame
        self._kick_dir_odom: Optional[float] = None
        # Last direction we could compute, held while a lookup transiently fails so
        # the command does not jump
        self._last_kick_dir_b = 0.0

        # The action runs in its own callback group so its execute callback can hold
        # the kick while the control loop keeps running
        self._action_server = ActionServer(
            self._node,
            Kick,
            str(node.get_parameter("command.action_name").value),
            execute_callback=self._execute_kick,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def has_data(self) -> bool:
        # Non-blocking: without a kick goal the policy is not run anyway
        return True

    def is_active(self) -> bool:
        """True while a kick goal is live"""
        return self._active

    def request_abort(self) -> None:
        """Ends the running kick, e.g. because we lost track of the ball"""
        self._abort_requested = True

    def get_kick_direction(self) -> np.ndarray:
        """Target kick direction as a unit vector in the yaw frame of the robot"""
        heading = self._compute_kick_dir_b()
        return np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)

    def get_kick_strong(self) -> np.ndarray:
        """Whether a strong (1.0) or a weak (0.0) kick was asked for"""
        return np.array([self._kick_strong], dtype=np.float32)

    def _compute_kick_dir_b(self) -> float:
        """Direction of the kick expressed in the current body frame"""
        if self._kick_dir_odom is None:
            return self._last_kick_dir_b
        yaw = yaw_in_frame(self._node, self._odom_frame, self._base_frame, timeout_s=0.0)
        if yaw is None:
            return self._last_kick_dir_b  # hold the last good value on a transient miss
        self._last_kick_dir_b = wrap_to_pi(self._kick_dir_odom - yaw)
        return self._last_kick_dir_b

    def _goal_callback(self, goal_request) -> GoalResponse:
        if self._active:
            self._node.get_logger().warning("A kick is already active; rejecting new kick goal.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_kick(self, goal_handle):
        goal = goal_handle.request
        timeout = float(goal.timeout) if goal.timeout > 0.0 else self._default_timeout
        result = Kick.Result()

        # (x, y) is the direction in the body frame at the time we received the goal
        self._kick_dir_body = math.atan2(float(goal.y), float(goal.x))
        self._kick_strong = 1.0 if float(goal.strength) >= self._strong_kick_min_strength else 0.0

        # Anchor the direction in odom so it stays fixed in the world while the robot
        # turns. Set the anchor before the kick goes active, so the control loop never
        # reads a half initialized anchor.
        anchor_yaw = yaw_in_frame(self._node, self._odom_frame, self._base_frame, timeout_s=0.1)
        if anchor_yaw is None:
            self._node.get_logger().warning("Could not anchor the kick direction in odom; aborting")
            goal_handle.abort()
            result.result = Kick.Result.ABORTED
            return result
        self._kick_dir_odom = wrap_to_pi(anchor_yaw + self._kick_dir_body)
        self._last_kick_dir_b = self._kick_dir_body
        self._abort_requested = False

        while not self._node.is_kickable():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result = Kick.Result.ABORTED
                return result
            self._node.get_logger().warn("Waiting for the robot to be controllable before starting the kick...")
            self._node.get_clock().sleep_for(Duration(seconds=0.02))

        self._active = True
        self._node.get_logger().info(
            f"Kick started: direction={math.degrees(self._kick_dir_body):.1f} deg (body), "
            f"strong={self._kick_strong > 0.0}, timeout={timeout:.2f} s"
        )

        feedback = Kick.Feedback()
        end = self._node.get_clock().now() + Duration(seconds=timeout)
        while self._node.get_clock().now() < end:
            if goal_handle.is_cancel_requested:
                self._active = False
                goal_handle.canceled()
                self._node.get_logger().info("Kick canceled.")
                result.result = Kick.Result.ABORTED
                return result
            if self._abort_requested:
                self._active = False
                goal_handle.abort()
                self._node.get_logger().warning("Kick aborted, as the ball is not available.")
                result.result = Kick.Result.ABORTED
                return result
            feedback.time_remaining = max(0.0, (end - self._node.get_clock().now()).nanoseconds / 1e9)
            goal_handle.publish_feedback(feedback)
            self._node.get_clock().sleep_for(Duration(seconds=0.05))

        self._active = False
        goal_handle.succeed()
        result.result = Kick.Result.SUCCESS
        self._node.get_logger().info("Kick finished.")
        return result
