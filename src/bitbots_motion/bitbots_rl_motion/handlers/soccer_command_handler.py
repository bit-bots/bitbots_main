import math
from typing import Optional

import numpy as np
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transform support
import tf2_ros
from bitbots_tf_buffer import Buffer
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time

from bitbots_msgs.action import Kick
from handlers.handler import Handler


def _wrap_to_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quat_xyzw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (rotation about z) from an xyzw quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


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
        body frame AT THE TIME THE GOAL IS RECEIVED (kick heading =
        ``atan2(y, x)``), a ``strength`` (kick-speed target) and a ``timeout``.
        That body-frame heading is anchored into the ``odom`` frame on receipt
        and re-expressed back into the current base frame every control step
        (``kick_dir_b = wrap(kick_dir_odom - robot_yaw_odom)``), so the kick
        keeps pointing at the same world direction even as the torso yaws during
        the kick -- matching the training/runner semantics. While the kick is
        active the velocity is forced to 0 (as in training) and the ball offset
        comes from ``ball_position_relative_filtered`` (transformed to base_footprint); the kick clears automatically after
        ``timeout`` seconds (``timeout < 0`` -> the configured default). When not
        kicking the ball is masked to 0, matching the walk role the policy was
        trained with.
    """

    def __init__(self, node):
        self._node = node

        self._default_kick_timeout = float(node.get_parameter("command.kick_timeout").value)
        self._warm_start_duration = float(node.get_parameter("command.warm_start_duration").value)
        self._post_kick_stand_duration = float(node.get_parameter("command.post_kick_stand_duration").value)
        self._pub_period = max(1, int(node.get_parameter("command.pub_period").value))
        self._history_samples = max(1, int(node.get_parameter("command.history_samples").value))
        self._warm_start_cmd = np.array(node.get_parameter("command.warm_start_command").value, dtype=np.float32)

        # Frames for anchoring the kick direction (REP-120). odom is the locally
        # consistent, drift-free-over-short-horizons world frame we anchor in.
        self._odom_frame = str(node.declare_parameter("odom_frame", "odom").value)
        self._base_footprint_frame = str(node.declare_parameter("base_footprint_frame", "base_footprint").value)
        # tf buffer to read the robot's yaw in odom (odom -> base_footprint).
        self._tf_buffer = Buffer(node=self._node)

        self._ball_pose: Optional[PoseWithCovarianceStamped] = None

        # kick state: set by the action (its own thread), read by the control loop
        self._warm_start_active = False
        self._kick_active = False
        self._kick_abort_requested = False
        self._post_kick_stand = False
        self._kick_speed = 0.0
        # Requested heading in the body frame at goal receipt (atan2(y, x)); used
        # as the fallback if the kick could not be anchored to odom.
        self._kick_dir_body = 0.0
        # The anchored heading in the odom frame (None if anchoring failed).
        self._kick_dir_odom: Optional[float] = None
        # Last successfully computed body-frame heading, held when a per-step tf
        # lookup transiently fails so the command does not jump.
        self._last_kick_dir_b = 0.0

        self._node.create_subscription(
            PoseWithCovarianceStamped, "ball_position_relative_filtered", self._ball_callback, 1
        )

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

    def _ball_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._ball_pose = msg

    def _get_ball_pos(self) -> Optional[np.ndarray]:
        if self._ball_pose is None:
            return None
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self._ball_pose.header.frame_id
        pose_stamped.header.stamp = Time(seconds=0).to_msg()
        pose_stamped.pose = self._ball_pose.pose.pose
        try:
            transformed = self._tf_buffer.transform(pose_stamped, "base_footprint", timeout=Duration(seconds=0.1))
            return np.array([transformed.pose.position.x, transformed.pose.position.y], dtype=np.float32)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self._node.get_logger().warn(f"Ball TF transform to base_footprint failed: {e}", throttle_duration_sec=1.0)
            return None

    def has_data(self) -> bool:
        # Non-blocking: missing cmd_vel/ball just yield zeros, so the policy can
        # run (standing) before any command or ball is received.
        return True

    # ------------------------------------------------------------------ #
    # kick action
    # ------------------------------------------------------------------ #
    def _goal_callback(self, goal_request) -> GoalResponse:
        if self._warm_start_active or self._kick_active:
            self._node.get_logger().warning("A kick is already active; rejecting new kick goal.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _robot_yaw_odom(self, timeout_s: float) -> Optional[float]:
        """Current robot yaw in the odom frame, or None if tf is unavailable.

        Reads the ``odom -> base_footprint`` transform; its rotation about z is
        the robot heading in odom.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_footprint_frame,
                Time(),
                Duration(seconds=timeout_s),
            )
        except Exception as e:  # tf2 raises various Lookup/Extrapolation errors
            self._node.get_logger().warning(f"Could not look up robot yaw in odom: {e}", throttle_duration_sec=1.0)
            return None
        q = tf.transform.rotation
        return _yaw_from_quat_xyzw(q.x, q.y, q.z, q.w)

    def _compute_kick_dir_b(self) -> float:
        """Kick heading expressed in the current base frame.

        Re-projects the odom-anchored heading into the current base frame each
        step. Falls back to the body-frame heading recorded at receipt if the
        kick was never anchored, and holds the last good value if a per-step
        lookup transiently fails.
        """
        if self._kick_dir_odom is None:
            return self._kick_dir_body  # never anchored -> body-fixed heading
        yaw_now = self._robot_yaw_odom(timeout_s=0.0)
        if yaw_now is None:
            return self._last_kick_dir_b  # hold last good value on a transient miss
        self._last_kick_dir_b = _wrap_to_pi(self._kick_dir_odom - yaw_now)
        return self._last_kick_dir_b

    def _execute_kick(self, goal_handle):
        """Hold the kick active for the requested timeout, then clear it.

        Runs in the action server's reentrant callback group (i.e. its own
        thread under a MultiThreadedExecutor) so the 50 Hz control loop keeps
        running while the kick is held.
        """
        goal = goal_handle.request
        timeout = float(goal.timeout) if goal.timeout > 0.0 else self._default_kick_timeout

        # (x, y) is the kick direction in the body frame at receipt -> heading.
        self._kick_dir_body = math.atan2(float(goal.y), float(goal.x))
        self._kick_speed = float(goal.strength)

        # Anchor the body-frame heading into odom so it stays world-fixed for the
        # duration of the kick. If tf is unavailable, fall back to a body-fixed
        # heading (kick_dir_odom = None). Set the anchor BEFORE marking the kick
        # active so the control loop never reads a half-initialized anchor.
        anchor_yaw = self._robot_yaw_odom(timeout_s=0.1)
        if anchor_yaw is not None:
            self._kick_dir_odom = _wrap_to_pi(anchor_yaw + self._kick_dir_body)
        else:
            self._kick_dir_odom = None
            self._node.get_logger().warning("Could not anchor kick to odom; using body-fixed kick direction.")
        self._last_kick_dir_b = self._kick_dir_body
        self._kick_abort_requested = False

        # Warm start: run the policy with a zero command so it can settle into a
        # neutral stance before the kick command becomes visible in the observation.
        if self._warm_start_duration > 0.0:
            self._warm_start_active = True
            warm_end = self._node.get_clock().now() + Duration(seconds=self._warm_start_duration)
            while self._node.get_clock().now() < warm_end:
                if goal_handle.is_cancel_requested:
                    self._warm_start_active = False
                    goal_handle.canceled()
                    result = Kick.Result()
                    result.result = Kick.Result.ABORTED
                    return result
                self._node.get_clock().sleep_for(Duration(seconds=0.02))
            self._warm_start_active = False

        self._kick_active = True
        self._node.get_logger().info(
            f"Kick started: dir={math.degrees(self._kick_dir_body):.1f} deg (body), "
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
            if self._kick_abort_requested:
                self._kick_active = False
                goal_handle.abort()
                self._node.get_logger().warning("Kick aborted: ball unavailable.")
                result.result = Kick.Result.ABORTED
                return result
            remaining = (end - self._node.get_clock().now()).nanoseconds / 1e9
            feedback.time_remaining = max(0.0, remaining)
            goal_handle.publish_feedback(feedback)
            self._node.get_clock().sleep_for(Duration(seconds=0.05))

        # disable kick and set cmd_vel to 0 so the policy does not try to walk while the kick is finishing
        self._kick_active = False

        # Post-kick standing
        if self._post_kick_stand_duration > 0.0:
            self._post_kick_stand = True
            post_kick_stand_end = self._node.get_clock().now() + Duration(seconds=self._post_kick_stand_duration)
            while self._node.get_clock().now() < post_kick_stand_end:
                remaining = (post_kick_stand_end - self._node.get_clock().now()).nanoseconds / 1e9
                self._node.get_logger().warn(f"Post-kick stand active remaining {remaining:.2f} s")
                feedback.time_remaining = max(0.0, remaining)
                goal_handle.publish_feedback(feedback)
                self._node.get_clock().sleep_for(Duration(seconds=0.05))
            self._post_kick_stand = False

        goal_handle.succeed()
        result.result = Kick.Result.SUCCESS
        self._node.get_logger().info("Kick finished.")
        return result

    # ------------------------------------------------------------------ #
    # per-step command construction
    # ------------------------------------------------------------------ #
    def _build_soccer_cmd(self) -> np.ndarray:
        if self._kick_active:
            vx, vy, wz = 0.0, 0.0, 0.0
            ball = self._get_ball_pos()
            if ball is not None:
                ball_x, ball_y = float(ball[0]), float(ball[1])
            else:
                ball_x, ball_y = 0.0, 0.0
                self._kick_abort_requested = True
            kick_dir = self._compute_kick_dir_b()
            kick_speed = self._kick_speed
        elif self._warm_start_active:
            vx, vy, wz = self._warm_start_cmd[0], self._warm_start_cmd[1], self._warm_start_cmd[2]
            ball_x, ball_y = 0.0, 0.0
            kick_dir = 0.0
            kick_speed = 0.0
        else:
            vx, vy, wz = 0.0, 0.0, 0.0
            ball_x, ball_y = 0.0, 0.0
            kick_dir = 0.0
            kick_speed = 0.0

        return np.array([vx, vy, wz, ball_x, ball_y, kick_dir, kick_speed], dtype=np.float32)

    def is_kick_active(self) -> bool:
        """True while an rl_kick action goal is live (warm start, kick, or post-kick stand).
        Used to gate when the policy runs and publishes."""
        return self._warm_start_active or self._kick_active or self._post_kick_stand

    def is_post_kick_stand(self) -> bool:
        """True during the post-kick standup window after the kick motion completes."""
        return self._post_kick_stand

    def reset(self) -> None:
        """Clear the command/ball histories so they are rebuilt on the next update.

        Called when the policy (re)activates so the histories do not carry stale
        pre-activation snapshots into the freshly started episode.
        """
        self._clean_hist = None
        self._soccer_hist = None
        self._pub_counter = 0

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
