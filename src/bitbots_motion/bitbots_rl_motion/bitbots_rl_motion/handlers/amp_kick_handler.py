import math
from collections.abc import Callable
from typing import Optional

import numpy as np
from bitbots_tf_buffer import Buffer
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time

from bitbots_msgs.action import Kick
from bitbots_rl_motion.handlers import Handler
from bitbots_rl_motion.handlers.soccer_command_handler import _wrap_to_pi, _yaw_from_quat_xyzw
from bitbots_rl_motion.piplus_amp_utils import kick_intensity_from_strength


class AmpKickHandler(Handler):
    """Runs the kick action and gives the task command of the Pi Plus AMP kick policy.

    The interface is the same rl_kick action (bitbots_msgs/Kick) as for the
    other kick policies:

      * The goal (x, y) is the kick direction in the body frame when the goal
        arrives. The handler anchors this direction in the odom frame and
        expresses it again in the current yaw frame of the robot at each step.
        Thus the direction stays fixed in the world while the robot turns.
        The policy observes it as the unit vector (cos, sin).
      * The goal strength is the target ball speed in m/s. The handler converts
        it to the kick intensity in [0, 1] with the inverse of the training map.
      * The kick stays active until the timeout of the goal. The policy has no
        "kick done" signal.
      * The handler rejects a goal if the node did not receive a ball estimate
        since it started. Thus the policy never starts without any ball.
    """

    def __init__(self, node, tf_buffer: Buffer, ball_received: Callable[[], bool]):
        self._node = node
        self._tf_buffer = tf_buffer
        # Returns True if the node received at least one ball estimate
        self._ball_received = ball_received

        self._default_timeout = float(node.get_parameter("command.kick_timeout").value)
        self._max_intensity = float(node.get_parameter("command.max_kick_intensity").value)
        self._odom_frame = str(node.get_parameter("command.odom_frame").value)
        self._base_frame = str(node.get_parameter("command.base_frame").value)

        # The action thread writes these values and the control loop reads them
        self._active = False
        self._abort_requested = False
        self._kick_intensity = 0.0
        self._kick_dir_odom: Optional[float] = None
        # Last direction in the body frame. Used if a transform lookup fails.
        self._last_kick_dir_b = 0.0

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
        # Do not block the node. The policy runs only during a kick goal.
        return True

    def is_active(self) -> bool:
        """Returns True while a kick goal is active."""
        return self._active

    def request_abort(self) -> None:
        """Stops the active kick, e.g. because the ball is lost."""
        self._abort_requested = True

    def get_kick_direction(self) -> np.ndarray:
        """Returns the kick direction as a unit vector in the yaw frame of the robot."""
        heading = self._compute_kick_dir_b()
        return np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)

    def get_kick_intensity(self) -> np.ndarray:
        return np.array([self._kick_intensity], dtype=np.float32)

    def _robot_yaw_odom(self, timeout_s: float) -> Optional[float]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time(), Duration(seconds=timeout_s)
            )
        except Exception as e:  # tf2 raises different lookup and extrapolation errors
            self._node.get_logger().warning(f"Could not look up the robot yaw in odom: {e}", throttle_duration_sec=1.0)
            return None
        q = transform.transform.rotation
        return _yaw_from_quat_xyzw(q.x, q.y, q.z, q.w)

    def _compute_kick_dir_b(self) -> float:
        if self._kick_dir_odom is None:
            return self._last_kick_dir_b
        yaw = self._robot_yaw_odom(timeout_s=0.0)
        if yaw is None:
            # Keep the last value if one lookup fails, so that the command does not jump
            return self._last_kick_dir_b
        self._last_kick_dir_b = _wrap_to_pi(self._kick_dir_odom - yaw)
        return self._last_kick_dir_b

    def _goal_callback(self, goal_request) -> GoalResponse:
        if self._active:
            self._node.get_logger().warning("A kick is already active. Reject the new kick goal.")
            return GoalResponse.REJECT
        if not self._ball_received():
            self._node.get_logger().warning("Kick goal rejected because no ball estimate was received yet.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_kick(self, goal_handle):
        goal = goal_handle.request
        timeout = float(goal.timeout) if goal.timeout > 0.0 else self._default_timeout
        result = Kick.Result()

        kick_dir_body = math.atan2(float(goal.y), float(goal.x))
        self._kick_intensity = kick_intensity_from_strength(float(goal.strength), self._max_intensity)

        # Set the anchor before the kick becomes active, so that the control loop
        # never reads an anchor that is not complete.
        anchor_yaw = self._robot_yaw_odom(timeout_s=0.1)
        if anchor_yaw is None:
            self._node.get_logger().warning("Could not anchor the kick direction in odom. Abort the kick.")
            goal_handle.abort()
            result.result = Kick.Result.ABORTED
            return result
        self._kick_dir_odom = _wrap_to_pi(anchor_yaw + kick_dir_body)
        self._last_kick_dir_b = kick_dir_body
        self._abort_requested = False

        while not self._node.is_kickable():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result = Kick.Result.ABORTED
                return result
            self._node.get_logger().warning(
                "Wait until the robot is controllable before the kick starts.", throttle_duration_sec=1.0
            )
            self._node.get_clock().sleep_for(Duration(seconds=0.02))

        self._active = True
        self._node.get_logger().info(
            f"Kick started: direction={math.degrees(kick_dir_body):.1f} deg (body), "
            f"intensity={self._kick_intensity:.2f}, timeout={timeout:.2f} s"
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
                self._node.get_logger().warning("Kick aborted, because the ball is not available.")
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
