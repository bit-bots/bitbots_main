import math
from typing import Optional

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from soccer_vision_3d_msgs.msg import BallArray

from bitbots_msgs.action import Kick
from handlers.handler import Handler


class SoccerCommandHandler(Handler):
    """Builds the soccer policy's command terms from the bitbots topic world.

    The encdec soccer policy consumes the command as TWO observation terms that
    are both derived from one 7-D snapshot per control step
    ``[vx, vy, wz, ball_x_b, ball_y_b, kick_dir_b, kick_speed]`` (robot yaw-base
    frame):

      * ``soccer_command``   : the 7-D snapshot with the ball columns [3:5]
        deleted -> 5-D ``[vx, vy, wz, kick_dir, kick_speed]``, kept as a
        ``history_samples`` history pushed EVERY step (50 Hz "clean" history).
      * ``ball_pos_b_noisy`` : the ball columns [3:5] -> 2-D ``[ball_x, ball_y]``,
        kept as a ``history_samples`` history pushed only every ``pub_period``
        steps (the 10 Hz async ball history fed to the encoder).

    Wiring (chosen for the bitbots stack):
      * velocity ``[vx, vy, wz]`` comes from ``cmd_vel``.
      * the kick is triggered by the ``rl_kick`` action (``bitbots_msgs/Kick``).
        The goal gives the kick direction as an ``(x, y)`` vector in the robot
        body frame (kick heading = ``atan2(y, x)``), a ``strength`` (kick-speed
        target) and a ``timeout``. While the kick is active the velocity is
        forced to 0 (as in training) and the ball offset comes from
        ``balls_relative``; the kick clears automatically after ``timeout``
        seconds (``timeout < 0`` -> the configured default). When not kicking
        the ball is masked to 0, matching the walk role the policy was trained
        with.
    """

    def __init__(self, node):
        self._node = node

        self._default_kick_timeout = float(node.get_parameter("command.kick_timeout").value)
        self._pub_period = max(1, int(node.get_parameter("command.pub_period").value))
        self._history_samples = max(1, int(node.get_parameter("command.history_samples").value))

        self._cmd_vel: Optional[Twist] = None
        self._ball_pos: Optional[np.ndarray] = None

        # kick state: set by the action (its own thread), read by the control loop
        self._kick_active = False
        self._kick_dir = 0.0
        self._kick_speed = 0.0

        self._node.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 1)
        self._node.create_subscription(BallArray, "balls_relative", self._ball_callback, 1)

        # The kick action runs in its own (reentrant) callback group so its
        # timed execute callback can run concurrently with the control loop.
        self._kick_action_server = ActionServer(
            self._node,
            Kick,
            "rl_kick",
            execute_callback=self._execute_kick,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # history state (newest at index -1), lazily initialized on first update
        self._clean_hist: Optional[np.ndarray] = None  # (H, 7), pushed every step
        self._soccer_hist: Optional[np.ndarray] = None  # (H, 7), pushed every pub_period
        self._pub_counter = 0

    # ------------------------------------------------------------------ #
    # subscriptions
    # ------------------------------------------------------------------ #
    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._cmd_vel = msg

    def _ball_callback(self, msg: BallArray) -> None:
        if msg.balls:
            center = msg.balls[0].center
            self._ball_pos = np.array([center.x, center.y], dtype=np.float32)

    def has_data(self) -> bool:
        # Non-blocking: missing cmd_vel/ball just yield zeros, so the policy can
        # run (standing) before any command or ball is received.
        return True

    # ------------------------------------------------------------------ #
    # kick action
    # ------------------------------------------------------------------ #
    def _goal_callback(self, goal_request) -> GoalResponse:
        if self._kick_active:
            self._node.get_logger().warning("A kick is already active; rejecting new kick goal.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_kick(self, goal_handle):
        """Hold the kick active for the requested timeout, then clear it.

        Runs in the action server's reentrant callback group (i.e. its own
        thread under a MultiThreadedExecutor) so the 50 Hz control loop keeps
        running while the kick is held.
        """
        goal = goal_handle.request
        timeout = float(goal.timeout) if goal.timeout > 0.0 else self._default_kick_timeout

        # (x, y) is the kick direction in the body frame -> heading angle.
        self._kick_dir = math.atan2(float(goal.y), float(goal.x))
        self._kick_speed = float(goal.strength)
        self._kick_active = True
        self._node.get_logger().info(
            f"Kick started: dir={math.degrees(self._kick_dir):.1f} deg, "
            f"strength={self._kick_speed:.2f}, timeout={timeout:.2f} s"
        )

        end = self._node.get_clock().now() + Duration(seconds=timeout)
        feedback = Kick.Feedback()
        result = Kick.Result()

        while self._node.get_clock().now() < end:
            if goal_handle.is_cancel_requested:
                self._kick_active = False
                goal_handle.canceled()
                self._node.get_logger().info("Kick canceled.")
                result.result = Kick.Result.ABORTED
                return result
            remaining = (end - self._node.get_clock().now()).nanoseconds / 1e9
            feedback.time_remaining = max(0.0, remaining)
            goal_handle.publish_feedback(feedback)
            # Sleep on the node clock so the wait follows ROS time (e.g. sim time)
            # rather than wall-clock time.
            self._node.get_clock().sleep_for(Duration(seconds=0.05))

        self._kick_active = False
        goal_handle.succeed()
        result.result = Kick.Result.SUCCESS
        self._node.get_logger().info("Kick finished.")
        return result

    # ------------------------------------------------------------------ #
    # per-step command construction
    # ------------------------------------------------------------------ #
    def _build_soccer_cmd(self) -> np.ndarray:
        if self._cmd_vel is not None:
            cmd_vx = float(self._cmd_vel.linear.x)
            cmd_vy = float(self._cmd_vel.linear.y)
            cmd_wz = float(self._cmd_vel.angular.z)
        else:
            cmd_vx = cmd_vy = cmd_wz = 0.0

        if self._kick_active:
            vx, vy, wz = 0.0, 0.0, 0.0
            if self._ball_pos is not None:
                ball_x, ball_y = float(self._ball_pos[0]), float(self._ball_pos[1])
            else:
                ball_x, ball_y = 0.0, 0.0
            kick_dir = self._kick_dir
            kick_speed = self._kick_speed
        else:
            vx, vy, wz = cmd_vx, cmd_vy, cmd_wz
            ball_x, ball_y = 0.0, 0.0
            kick_dir = 0.0
            kick_speed = 0.0

        return np.array([vx, vy, wz, ball_x, ball_y, kick_dir, kick_speed], dtype=np.float32)

    def update(self) -> None:
        """Advance both history buffers by one control step. Call once per step."""
        cmd7 = self._build_soccer_cmd()

        if self._clean_hist is None:
            self._clean_hist = np.tile(cmd7, (self._history_samples, 1)).astype(np.float32)
            self._soccer_hist = np.tile(cmd7, (self._history_samples, 1)).astype(np.float32)
            self._pub_counter = 0

        # vel/kick (cols 0:3, 5:7): 50 Hz real history, shift+push every step
        self._clean_hist[:-1] = self._clean_hist[1:]
        self._clean_hist[-1] = cmd7

        # ball xy (cols 3:5): 10 Hz async history, shift+push only every pub_period steps
        if self._pub_counter % self._pub_period == 0:
            self._soccer_hist[:-1] = self._soccer_hist[1:]
            self._soccer_hist[-1] = cmd7
        self._pub_counter += 1

    def get_soccer_command_obs(self) -> np.ndarray:
        """5-D-per-frame command history (ball columns deleted), flattened."""
        assert self._clean_hist is not None, "update() must be called before reading the command"
        out = np.delete(self._clean_hist, [3, 4], axis=1)  # (H, 5)
        return out.reshape(-1).copy()

    def get_ball_obs(self) -> np.ndarray:
        """Ball xy history fed to the encoder, flattened."""
        assert self._soccer_hist is not None, "update() must be called before reading the ball"
        return self._soccer_hist[:, 3:5].reshape(-1).copy()
